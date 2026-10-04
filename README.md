# Đồ án: Hệ thống IoT Cảnh báo Té ngã cho Người già & Ứng dụng Flutter

Dự án này là mã nguồn hoàn chỉnh của hệ thống cảnh báo té ngã, bao gồm firmware chạy trên vi điều khiển ESP32, ứng dụng giám sát trên điện thoại (Flutter), quy tắc bảo mật cơ sở dữ liệu (Firebase) và bộ mô phỏng thuật toán trên máy tính (C++).

## Cấu trúc thư mục

```text
D:\Mpu6050\
├── app/
│   └── main_v2.dart                   # Mã nguồn ứng dụng di động (Flutter)
├── firebase/
│   └── database_rules_v2.json         # Quy tắc bảo mật Firebase Realtime Database
├── firmware/
│   └── Mpu6050/
│       └── Mpu6050.ino                # Mã nguồn chính nạp vào ESP32
├── sim/
│   ├── fw_core.inc                    # Lõi thuật toán phát hiện té ngã
│   ├── mock.h                         # Thư viện giả lập phần cứng
│   └── sim_main.cpp                   # Chương trình mô phỏng & kiểm thử
├── Nguyen Huu Hung Vip_3.docx         # Báo cáo chi tiết đồ án
└── README.md                          # File hướng dẫn (bạn đang đọc)
1. Thiết lập cơ sở dữ liệu (Firebase)
Tạo dự án trên Firebase Console.

Bật Authentication, cho phép phương thức đăng nhập Anonymous (Ẩn danh).

Tạo Realtime Database, vào tab Rules (Quy tắc).

Mở file firebase/database_rules_v2.json, copy toàn bộ nội dung dán vào và bấm Publish.

2. Hướng dẫn nạp code vào mạch ESP32 (Chi tiết)
Phần này hướng dẫn nạp file Mpu6050.ino vào mạch ESP32 thật.

Bước 1: Chuẩn bị phần cứng & Cài đặt môi trường

Dùng cáp USB kết nối mạch ESP32 với máy tính.

Mở phần mềm Arduino IDE.

Cài đặt gói hỗ trợ ESP32: Vào File > Preferences, dán link https://dl.espressif.com/dl/package_esp32_index.json vào ô Additional Boards Manager URLs. Sau đó vào Tools > Board > Boards Manager..., tìm từ khóa esp32 và bấm Install.

Bước 2: Cài đặt các thư viện bắt buộc

Trong Arduino IDE, vào Sketch > Include Library > Manage Libraries...

Tìm và cài đặt thư viện MPU6050_light (của rfetick).

Tìm và cài đặt thư viện WiFiManager (của tzapu).

Bước 3: Mở file và Chọn cổng kết nối

Vào File > Open... và trỏ tới file firmware/Mpu6050/Mpu6050.ino.

Chọn loại mạch: Vào Tools > Board > ESP32 Arduino > chọn ESP32 Dev Module.

Chọn cổng kết nối: Vào Tools > Port và tích chọn cổng COM tương ứng với mạch (ví dụ: COM3, COM4).

Bước 4: Nạp code (Upload)

Bấm vào nút Upload (biểu tượng mũi tên ngang góc trên bên trái).

Chờ phần mềm biên dịch và nạp. Khi dòng chữ phía dưới báo "Done uploading." là thành công.

Bước 5: Cấu hình Wi-Fi lần đầu cho mạch

Khi mạch vừa nạp xong, nó sẽ phát ra một mạng Wi-Fi tên là FallDetector-Setup.

Dùng điện thoại kết nối vào Wi-Fi này. Một trang web sẽ tự động bật lên.

Bấm vào Configure WiFi, chọn tên Wi-Fi nhà bạn, nhập mật khẩu và bấm Save.

Mạch ESP32 sẽ tự khởi động lại và kết nối vào mạng nhà bạn để bắt đầu gửi dữ liệu.

3. Hướng dẫn chạy mô phỏng kiểm thử trên OnlineGDB (Chi tiết)
Đây là cách để bạn chạy giả lập luồng thuật toán 4 pha (dùng các file trong thư mục sim/) trực tiếp trên trình duyệt web mà không cần cài phần mềm.

Bước 1: Mở trình duyệt và cài đặt ngôn ngữ

Truy cập vào trang web: https://www.onlinegdb.com/

Ở góc trên cùng bên phải, tìm ô Language, xổ xuống và chọn C++.

Bước 2: Chuẩn bị file chính (main.cpp)

Ở cửa sổ soạn thảo giữa màn hình (đang có sẵn file tên là main.cpp), xóa toàn bộ code mẫu có sẵn đi.

Mở file sim/sim_main.cpp trong thư mục đồ án trên máy tính của bạn, copy toàn bộ nội dung và dán vào tab main.cpp trên web.

Bước 3: Tạo file giả lập phần cứng (mock.h)

Nhìn sang cột bên trái (Project), bấm vào biểu tượng hình tờ giấy có dấu cộng (New File).

Gõ chính xác tên file là: mock.h rồi ấn Enter.

Mở file sim/mock.h trên máy tính, copy toàn bộ và dán vào tab mock.h trên web.

Bước 4: Tạo file lõi thuật toán (fw_core.inc)

Tiếp tục bấm vào biểu tượng New File lần nữa.

Gõ chính xác tên file là: fw_core.inc rồi ấn Enter.

Mở file sim/fw_core.inc trên máy tính, copy toàn bộ và dán vào tab fw_core.inc trên web.

(Đến đây, bạn phải thấy ở cột Project bên trái có đúng 3 file: main.cpp, mock.h, fw_core.inc)

Bước 5: Chạy kiểm thử

Bấm nút Run (Màu xanh lá, có hình tam giác) ở thanh công cụ phía trên.

Đợi 2-3 giây, hệ thống sẽ chạy 45 ca kiểm thử và in kết quả hiển thị màu sắc rõ ràng (PASS/FAIL) ở cửa sổ Console (khu vực viền đen) phía dưới cùng màn hình.

4. Chạy ứng dụng điện thoại (Flutter App)
Copy file app/main_v2.dart đè lên file main.dart trong thư mục lib/ của project Flutter gốc.

Đảm bảo máy tính đã cài đặt Flutter SDK và cấu hình Firebase (lệnh flutterfire configure).

Mở Terminal tại thư mục project Flutter, kết nối điện thoại (hoặc bật máy ảo) và gõ lệnh:

Bash
flutter run
Khi ứng dụng lên màn hình, nó sẽ tự động kết nối Firebase và nhận cảnh báo khi mạch phát hiện sự cố.
