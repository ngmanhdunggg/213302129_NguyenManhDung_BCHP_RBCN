BÀI 1 – MÔ PHỎNG ĐỘNG HỌC & QUỸ ĐẠO ROBOT 6 DOF (MATLAB)

Chương trình mô phỏng robot công nghiệp 6 bậc tự do với khả năng lập kế hoạch quỹ đạo và điều khiển mượt bằng hồ sơ S-curve. Hệ thống cho phép khai báo danh sách waypoint, lựa chọn kiểu di chuyển MoveL (đường thẳng) hoặc MoveJ (không gian khớp), đồng thời thiết lập vận tốc – gia tốc riêng cho từng đoạn.

Robot được xây dựng từ mô hình Denavit–Hartenberg, kết hợp thuật toán động học nghịch tùy chỉnh để tính các góc khớp tương ứng với từng vị trí TCP trong không gian. Chương trình tích hợp cơ chế kiểm tra an toàn, bao gồm giới hạn khớp, tránh vùng cấm và đánh giá sai lệch vị trí TCP, đảm bảo mọi quỹ đạo được sinh ra đều hợp lệ.

Toàn bộ chuyển động được mô phỏng trực quan bằng Robotics Toolbox, cho phép quan sát cấu hình robot và đường đi TCP trong không gian 3D. Mô hình này hỗ trợ khảo sát quỹ đạo kiểm tra bốn góc của miếng gỗ ép, đồng thời dễ dàng mở rộng cho các bài toán điều khiển robot trong môi trường công nghiệp.

BÀI 2 – LẬP TRÌNH UR3 TRONG URSIM KIỂM TRA 4 GÓC MIẾNG GỖ

Chương trình URScript được xây dựng cho robot UR3 nhằm thực hiện kiểm tra chất lượng sản phẩm bằng camera gắn tại TCP. Bài toán yêu cầu robot tiếp cận miếng gỗ vuông, di chuyển lần lượt đến bốn vị trí góc và chụp ảnh để đánh giá độ chính xác biên dạng và chất lượng sơn tại từng điểm.

Hệ thống sử dụng cảm biến số để phát hiện sản phẩm đến vị trí kiểm tra, kết hợp các tín hiệu xuất để kích hoạt camera và báo hoàn thành chu kỳ. Robot được lập trình di chuyển qua các điểm: Home → điểm tiếp cận → bốn góc P1–P4 → về lại Home, với phân tách rõ giữa chuyển động nhanh (MoveJ) và chuyển động quan sát ổn định (MoveL).

Quá trình mô phỏng trong URSim thể hiện đầy đủ chu trình kiểm tra 20 sản phẩm, bao gồm nhận tín hiệu cảm biến, di chuyển an toàn, chụp ảnh từng góc, thông báo kết quả và chờ sản phẩm rời khỏi vùng kiểm tra. Mô hình phản ánh đúng một chu trình kiểm tra chất lượng trong dây chuyền sản xuất gỗ ép.
