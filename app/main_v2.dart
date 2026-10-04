// =============================================================================
//  ỨNG DỤNG CẢNH BÁO TÉ NGÃ CHO NGƯỜI THÂN - v2.0
//  Đồng bộ với firmware FallDetector_ESP32 v2.0
//
//  Thay đổi so với v1:
//   - Lược đồ dữ liệu đa thiết bị: /devices/<deviceId>/{status,command,events}
//   - Giao thức lệnh idempotent: ghi {cmd, id} với id tăng dần theo giờ máy chủ
//   - Hiển thị nhật ký sự kiện té ngã gần nhất (đọc từ node events)
//   - Nút "Kiểm tra hệ thống" gửi lệnh TEST để diễn tập/nghiệm thu
//   - Hiển thị cường độ sóng Wi-Fi và phiên bản firmware của thiết bị
// =============================================================================

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_database/firebase_database.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'firebase_options.dart';

/// Mã thiết bị được theo dõi (trùng với device id cấu hình trên ESP32).
const String kDeviceId = 'device01';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await Firebase.initializeApp(options: DefaultFirebaseOptions.currentPlatform);
  // Không để lỗi đăng nhập (ví dụ đang offline) làm app không mở được.
  try {
    if (FirebaseAuth.instance.currentUser == null) {
      await FirebaseAuth.instance.signInAnonymously();
    }
  } catch (e) {
    debugPrint('Lỗi đăng nhập ẩn danh: $e');
  }
  runApp(const DoAnKyApp());
}

class DoAnKyApp extends StatelessWidget {
  const DoAnKyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Đồ Án Kỳ',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.indigo),
      ),
      home: const FallDetectionScreen(),
    );
  }
}

/// Một bản ghi sự kiện té ngã đọc từ node events.
class FallEventRecord {
  final String trigger;
  final double peakAcc;
  final double peakGyro;
  final double tiltChange;
  final int atMs;

  FallEventRecord({
    required this.trigger,
    required this.peakAcc,
    required this.peakGyro,
    required this.tiltChange,
    required this.atMs,
  });

  factory FallEventRecord.fromMap(Map data) {
    double num2d(Object? v) => v is num ? v.toDouble() : 0;
    return FallEventRecord(
      trigger: data['trigger']?.toString() ?? 'FREEFALL',
      peakAcc: num2d(data['peakAcc']),
      peakGyro: num2d(data['peakGyro']),
      tiltChange: num2d(data['tiltChange']),
      atMs: data['at'] is num ? (data['at'] as num).toInt() : 0,
    );
  }

  String get triggerLabel {
    switch (trigger) {
      case 'HARD_IMPACT':
        return 'Va chạm mạnh';
      case 'SOS':
        return 'SOS (giữ nút)';
      case 'TEST':
        return 'Kiểm tra hệ thống';
      default:
        return 'Ngã có pha rơi';
    }
  }
}

class FallDetectionScreen extends StatefulWidget {
  const FallDetectionScreen({super.key});

  @override
  State<FallDetectionScreen> createState() => _FallDetectionScreenState();
}

class _FallDetectionScreenState extends State<FallDetectionScreen> {
  // Thiết bị gửi heartbeat mỗi 10 s. Quá 35 s không có dữ liệu mới => mất kết nối.
  static const int _offlineAfterMs = 35000;

  final DatabaseReference _statusRef =
      FirebaseDatabase.instance.ref('devices/$kDeviceId/status');
  final DatabaseReference _cmdRef =
      FirebaseDatabase.instance.ref('devices/$kDeviceId/command');
  final Query _eventsQuery = FirebaseDatabase.instance
      .ref('devices/$kDeviceId/events')
      .orderByChild('at')
      .limitToLast(5);
  final DatabaseReference _offsetRef =
      FirebaseDatabase.instance.ref('.info/serverTimeOffset');

  StreamSubscription<DatabaseEvent>? _statusSub;
  StreamSubscription<DatabaseEvent>? _eventsSub;
  StreamSubscription<DatabaseEvent>? _offsetSub;
  Timer? _ticker;

  String currentState = 'IDLE';
  bool acked = false;
  double accTotal = 1.00;
  double gyroTotal = 0.0;
  double tiltAngle = 0.0;
  int? rssi;
  String? firmware;
  int? lastSeenMs; // giờ server Firebase (ms) của lần thiết bị gửi gần nhất
  int _serverOffsetMs = 0; // chênh lệch giữa đồng hồ điện thoại và server
  bool hasReceivedData = false;
  bool _listenError = false;
  bool _sending = false;
  List<FallEventRecord> _events = [];

  @override
  void initState() {
    super.initState();

    _offsetSub = _offsetRef.onValue.listen((DatabaseEvent event) {
      final v = event.snapshot.value;
      if (v is num) _serverOffsetMs = v.toInt();
    }, onError: (Object error) {
      debugPrint('Lỗi đọc serverTimeOffset: $error');
    });

    _subscribe();

    // Mỗi 5 s: đánh giá lại "mất kết nối" và tự nghe lại nếu luồng bị ngắt.
    _ticker = Timer.periodic(const Duration(seconds: 5), (_) => _tick());
  }

  @override
  void dispose() {
    _ticker?.cancel();
    _statusSub?.cancel();
    _eventsSub?.cancel();
    _offsetSub?.cancel();
    super.dispose();
  }

  // ---------------- Kết nối dữ liệu ----------------

  void _subscribe() {
    _statusSub?.cancel();
    _statusSub = _statusRef.onValue.listen(_onStatus, onError: (Object error) {
      debugPrint('Lỗi đọc Firebase: $error');
      _statusSub?.cancel();
      _statusSub = null;
      if (mounted) setState(() => _listenError = true);
    });

    _eventsSub?.cancel();
    _eventsSub = _eventsQuery.onValue.listen((DatabaseEvent event) {
      final data = event.snapshot.value;
      if (data is! Map || !mounted) return;
      final list = data.values
          .whereType<Map>()
          .map(FallEventRecord.fromMap)
          .toList()
        ..sort((a, b) => b.atMs.compareTo(a.atMs));
      setState(() => _events = list);
    }, onError: (Object error) {
      debugPrint('Lỗi đọc nhật ký sự kiện: $error');
    });
  }

  void _onStatus(DatabaseEvent event) {
    final data = event.snapshot.value;
    if (data == null || data is! Map) return;
    if (!mounted) return;

    num? asNum(Object? v) => v is num ? v : null;

    setState(() {
      currentState = data['state']?.toString() ?? currentState;
      acked = data['acked'] == true;
      accTotal = asNum(data['accTotal'])?.toDouble() ?? accTotal;
      gyroTotal = asNum(data['gyroTotal'])?.toDouble() ?? gyroTotal;
      tiltAngle = asNum(data['tiltAngle'])?.toDouble() ?? tiltAngle;
      rssi = asNum(data['rssi'])?.toInt() ?? rssi;
      firmware = data['fw']?.toString() ?? firmware;
      lastSeenMs = asNum(data['lastSeen'])?.toInt() ?? lastSeenMs;
      hasReceivedData = true;
      _listenError = false;
    });
  }

  Future<void> _ensureAuth() async {
    if (FirebaseAuth.instance.currentUser != null) return;
    try {
      await FirebaseAuth.instance.signInAnonymously();
    } catch (e) {
      debugPrint('Lỗi đăng nhập ẩn danh: $e');
    }
  }

  Future<void> _tick() async {
    if (_statusSub == null) {
      await _ensureAuth();
      if (!mounted) return;
      if (FirebaseAuth.instance.currentUser != null) _subscribe();
    }
    if (mounted) setState(() {}); // cập nhật trạng thái online/offline
  }

  // Gửi lệnh theo giao thức idempotent: {cmd, id} với id là mốc thời gian
  // máy chủ (ms). Thiết bị bỏ qua mọi id đã xử lý nên không cần ghi ngược.
  Future<void> _sendCommand(String cmd) async {
    if (_sending) return;
    setState(() => _sending = true);
    final int id = DateTime.now().millisecondsSinceEpoch + _serverOffsetMs;
    try {
      await _cmdRef
          .set({'cmd': cmd, 'id': id})
          .timeout(const Duration(seconds: 8));
    } catch (e) {
      debugPrint('Lỗi gửi lệnh: $e');
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
            content: Text('Không gửi được lệnh. Kiểm tra kết nối mạng.')),
      );
    } finally {
      if (mounted) setState(() => _sending = false);
    }
  }

  // ---------------- Tiện ích ----------------

  bool get _deviceOnline {
    final seen = lastSeenMs;
    if (seen == null) return false;
    final nowServer = DateTime.now().millisecondsSinceEpoch + _serverOffsetMs;
    return (nowServer - seen) <= _offlineAfterMs;
  }

  String _two(int n) => n.toString().padLeft(2, '0');

  String _formatTime(int? ms) {
    if (ms == null) return '--:--:--';
    final d = DateTime.fromMillisecondsSinceEpoch(ms).toLocal();
    return '${_two(d.hour)}:${_two(d.minute)}:${_two(d.second)}\n${_two(d.day)}/${_two(d.month)}';
  }

  String _formatDateTime(int ms) {
    if (ms == 0) return '--';
    final d = DateTime.fromMillisecondsSinceEpoch(ms).toLocal();
    return '${_two(d.day)}/${_two(d.month)} ${_two(d.hour)}:${_two(d.minute)}';
  }

  // ---------------- Giao diện ----------------

  @override
  Widget build(BuildContext context) {
    final bool isAlarm = currentState == 'ALARM';
    final bool isChecking = currentState == 'CHECKING';
    final bool online = _deviceOnline;
    final bool offline = hasReceivedData && !online;

    // Ưu tiên: ALARM > mất kết nối > đang kiểm tra > an toàn
    Color statusColor;
    String statusText;
    IconData statusIcon;
    if (isAlarm) {
      statusColor = Colors.red;
      statusText = acked
          ? 'ĐÃ TIẾP NHẬN CẢNH BÁO TÉ NGÃ'
          : 'CẢNH BÁO: TÉ NGÃ THẬT!';
      statusIcon = Icons.warning_amber_rounded;
    } else if (offline) {
      statusColor = Colors.blueGrey;
      statusText =
          'MẤT KẾT NỐI THIẾT BỊ\nKhông xác định được tình trạng an toàn';
      statusIcon = Icons.cloud_off;
    } else if (isChecking) {
      statusColor = Colors.orange;
      statusText = 'ĐANG PHÁT HIỆN TÍN HIỆU TÉ NGÃ...';
      statusIcon = Icons.sync;
    } else {
      statusColor = Colors.green;
      statusText = 'TRẠNG THÁI: AN TOÀN';
      statusIcon = Icons.check_circle_outline;
    }

    return Scaffold(
      backgroundColor: const Color(0xFFF8FAFC),
      appBar: AppBar(
        title: const Text('Đồ Án Kỳ - Phát Hiện Té Ngã',
            style: TextStyle(fontWeight: FontWeight.bold, fontSize: 18)),
        centerTitle: true,
        backgroundColor: Colors.white,
        elevation: 0,
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            if (!hasReceivedData)
              _infoBanner(_listenError
                  ? 'Không đọc được dữ liệu từ Firebase (kiểm tra mạng / đăng nhập). Ứng dụng sẽ tự thử lại.'
                  : 'Chưa nhận được dữ liệu từ thiết bị. Kiểm tra ESP32 đã kết nối WiFi chưa.'),
            if (isAlarm && offline)
              _infoBanner(
                  'Thiết bị đang mất kết nối - thông tin cảnh báo có thể đã cũ.'),
            _statusCard(statusColor, statusText, statusIcon, isAlarm),
            const SizedBox(height: 25),
            const Text(
              'Dữ liệu cảm biến MPU6050',
              style: TextStyle(
                  fontSize: 16,
                  fontWeight: FontWeight.bold,
                  color: Colors.black87),
            ),
            const SizedBox(height: 4),
            const Text(
              'Khi có sự cố, gia tốc và tốc độ xoay hiển thị giá trị đỉnh của sự kiện.',
              style: TextStyle(fontSize: 12, color: Colors.grey),
            ),
            const SizedBox(height: 12),
            GridView.count(
              shrinkWrap: true,
              physics: const NeverScrollableScrollPhysics(),
              crossAxisCount: 2,
              crossAxisSpacing: 15,
              mainAxisSpacing: 15,
              childAspectRatio: 1.15,
              children: [
                _buildSensorCard('Gia tốc tổng',
                    '${accTotal.toStringAsFixed(2)} g', Icons.speed, Colors.blue),
                _buildSensorCard('Tốc độ xoay',
                    '${gyroTotal.toStringAsFixed(1)} °/s',
                    Icons.screen_rotation, Colors.purple),
                _buildSensorCard('Góc nghiêng tư thế',
                    '${tiltAngle.toStringAsFixed(1)}°', Icons.navigation,
                    Colors.teal),
                _buildSensorCard('Cập nhật lúc', _formatTime(lastSeenMs),
                    Icons.access_time, online ? Colors.indigo : Colors.grey),
              ],
            ),
            const SizedBox(height: 20),
            _deviceInfoRow(online),
            const SizedBox(height: 20),
            _eventHistory(),
            const SizedBox(height: 10),
            if (!isAlarm)
              OutlinedButton.icon(
                onPressed: _sending ? null : () => _sendCommand('TEST'),
                icon: const Icon(Icons.play_circle_outline),
                label: const Text('Kiểm tra hệ thống (diễn tập cảnh báo)'),
              ),
          ],
        ),
      ),
    );
  }

  Widget _statusCard(
      Color statusColor, String statusText, IconData statusIcon, bool isAlarm) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.symmetric(vertical: 30, horizontal: 20),
      decoration: BoxDecoration(
        color: statusColor,
        borderRadius: BorderRadius.circular(24),
        boxShadow: [
          BoxShadow(
            color: statusColor.withValues(alpha: 0.3),
            blurRadius: 15,
            offset: const Offset(0, 8),
          )
        ],
      ),
      child: Column(
        children: [
          Icon(statusIcon, color: Colors.white, size: 70),
          const SizedBox(height: 15),
          Text(
            statusText,
            textAlign: TextAlign.center,
            style: const TextStyle(
                color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold),
          ),
          if (isAlarm) ...[
            const SizedBox(height: 15),
            ElevatedButton.icon(
              onPressed:
                  _sending ? null : () => _sendCommand(acked ? 'RESET' : 'ACK'),
              icon: Icon(
                acked ? Icons.check_circle_outline : Icons.notifications_off,
                color: Colors.red,
              ),
              label: Text(
                acked ? 'KẾT THÚC CẢNH BÁO' : 'ĐÃ XEM - TẮT CÒI',
                style: const TextStyle(
                    color: Colors.red, fontWeight: FontWeight.bold),
              ),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.white,
                padding:
                    const EdgeInsets.symmetric(horizontal: 20, vertical: 12),
              ),
            ),
            if (acked)
              const Padding(
                padding: EdgeInsets.only(top: 10),
                child: Text(
                  'Chỉ kết thúc khi đã chắc chắn người thân an toàn.',
                  textAlign: TextAlign.center,
                  style: TextStyle(color: Colors.white, fontSize: 12),
                ),
              ),
          ],
        ],
      ),
    );
  }

  Widget _deviceInfoRow(bool online) {
    final String signal = rssi == null ? '--' : '$rssi dBm';
    return Row(
      children: [
        Icon(online ? Icons.wifi : Icons.wifi_off,
            size: 18, color: online ? Colors.green : Colors.grey),
        const SizedBox(width: 6),
        Text('Thiết bị $kDeviceId · sóng $signal · firmware ${firmware ?? "--"}',
            style: const TextStyle(fontSize: 12, color: Colors.grey)),
      ],
    );
  }

  Widget _eventHistory() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Text('Nhật ký sự kiện gần nhất',
            style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.bold,
                color: Colors.black87)),
        const SizedBox(height: 8),
        if (_events.isEmpty)
          const Text('Chưa ghi nhận sự kiện nào.',
              style: TextStyle(fontSize: 13, color: Colors.grey))
        else
          ..._events.map((e) => Container(
                margin: const EdgeInsets.only(bottom: 8),
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(14),
                  border: Border.all(color: Colors.grey.shade200),
                ),
                child: Row(
                  children: [
                    const Icon(Icons.history, size: 20, color: Colors.indigo),
                    const SizedBox(width: 10),
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text('${e.triggerLabel} · ${_formatDateTime(e.atMs)}',
                              style: const TextStyle(
                                  fontSize: 13, fontWeight: FontWeight.bold)),
                          Text(
                            'Đỉnh ${e.peakAcc.toStringAsFixed(2)} g · '
                            '${e.peakGyro.toStringAsFixed(0)} °/s · '
                            'đổi tư thế ${e.tiltChange.toStringAsFixed(0)}°',
                            style: const TextStyle(
                                fontSize: 12, color: Colors.grey),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              )),
      ],
    );
  }

  Widget _infoBanner(String message) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(12),
      margin: const EdgeInsets.only(bottom: 15),
      decoration: BoxDecoration(
        color: Colors.amber.shade100,
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        children: [
          const Icon(Icons.info_outline, color: Colors.orange),
          const SizedBox(width: 10),
          Expanded(child: Text(message, style: const TextStyle(fontSize: 13))),
        ],
      ),
    );
  }

  Widget _buildSensorCard(
      String title, String value, IconData icon, Color color) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: Colors.grey.shade200),
      ),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(icon, color: color, size: 28),
          const SizedBox(height: 8),
          Text(title,
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.grey, fontSize: 13)),
          const SizedBox(height: 4),
          Text(
            value,
            style: const TextStyle(
                fontWeight: FontWeight.bold,
                fontSize: 15,
                color: Colors.black87),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }
}
