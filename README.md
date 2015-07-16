# RaiseYourArm2015
- Mỗi người chọn 1 cái local svn tool down về rồi sài (git, commit, sync source ...). Vd như github, tortoise git, ... và google để biết cách sử dụng.
- Lưu ý là source này nhiều người cùng viết nên cố gắng đóng kín tối đa từng module mình viết để tiện cho việc integrate sau này. Mỗi module sẽ có 1 hàm process chính, và được call tuần tự trong chương trình chính, vd:
    void main(void)
    {
        //Init
        ...
        while (1)
        {
            Module1_process();
            Module2_process();
        }
    }
Vì vậy nên trong mỗi module không dùng các hàm blocking, delay, chỉ dùng các biến tick sẽ được register vô 1 cái timercallback nào đấy để tick và delay ... tránh ở quá lâu trong 1 process để các module khác chạy
- Mỗi module đóng kín nên cần có các hàm get, set, tránh dùng các biến toàn cục. Cần quản lí các biến trong module. Cần thiết thì tạo các biến dynamic trên RAM, nếu có các biến tạm lưu trên stack thì cần nắm cái size cho các biến này.
- Cố gắng sài cái base này để viết thêm vô, không sửa các module khác để không ảnh hưởng process của người khác.
- Mỗi ngày cố gắng commit source lên để nắm process.
