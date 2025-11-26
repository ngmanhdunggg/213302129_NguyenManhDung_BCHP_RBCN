
Chương trình này mô phỏng robot công nghiệp 6 bậc tự do với khả năng lập kế hoạch quỹ đạo và điều khiển chuyển động mượt theo hồ sơ S-curve. Hệ thống cho phép khai báo danh sách các điểm di chuyển (waypoint), hỗ trợ di chuyển theo đường thẳng (MoveL) hoặc theo không gian khớp (MoveJ), kèm cấu hình vận tốc và gia tốc riêng cho từng bước.

Robot được xây dựng dựa trên mô hình DH và sử dụng thuật toán động học ngược tùy chỉnh để tính góc khớp cho từng vị trí trong không gian. Chương trình cũng tích hợp cơ chế kiểm tra an toàn, bao gồm giới hạn khớp, vùng không gian cấm và độ lệch TCP, giúp đảm bảo đường đi luôn hợp lệ.

Cuối cùng, toàn bộ quỹ đạo được mô phỏng trực quan bằng Robotics Toolbox, cho phép quan sát chi tiết chuyển động của robot trong không gian 3D.
