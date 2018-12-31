#include <ros/ros.h>  // rosで必要はヘッダーファイル
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <fcntl.h>
using namespace std;

#define MAX_VEL 0.5//m/s
#define MIN_VEL -0.5//m/s
#define MAX_ANGULAR_VEL 2.3//m/s
#define MIN_ANGULAR_VEL -2.3//m/s


int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        ch = EOF;
        return 1;
    }
    return 0;
}

char getch(){

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");
    return (buf);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_teleop_node");
    // initでROSを初期化し、my_teleop_nodeという名前をノードにつける
    // 同じ名前のノードが複数あるとだめなので、ユニークな名前をつける

    ros::NodeHandle nh;
    // ノードハンドラの作成。ハンドラは必要時に起動される。

    ros::Publisher  pub;
    // パブリッシャの作成。トピックに対してデータを送信。

    ros::Rate rate(10);
    // ループの頻度を設定するためのオブジェクトを作成。この場合は10Hz、1秒間に10回数、1ループ100ms。

    geometry_msgs::Twist cmd_vel;
    // geometry_msgs::Twist　この型は並進速度と回転速度(vector3:3次元ベクトル) を合わせたもので、速度指令によく使われる

    pub = nh.advertise<geometry_msgs::Twist>("/sub", 1);
    // マスターにgeometry_msgs::Twist型のデータを送ることを伝える
    // マスターは/cmd_velトピック(1番目の引数）を購読する
    // 全てのノードにトピックができたことを知らせる(advertise)。
    // 2番目の引数はデータのバッファサイズ

    cout << "w: forward, s: backward, a: left, d:right, z: left turn, x:right turn" << endl;
    //printf と同じ

    while (ros::ok()) { // このノードが使える間は無限ループする
        char key;  // 入力キーの値

    //ROS_INFO("[%c]",getch()); 


//            key = getch();

        while(kbhit() != 0) // 標準入力からキーを読み込む
        {
            key = getchar();
//            ROS_INFO("[%s]",key); 
            switch (key) {
            case 'w': // fキーが押されていたら
                cmd_vel.linear.x  =  MAX_VEL;
                break;
            case 's':
                cmd_vel.linear.x  =  MIN_VEL;
                break;
            case 'a':
                cmd_vel.linear.y =  MAX_VEL;
                break;
            case 'd':
                cmd_vel.linear.y = MIN_VEL;
                break;
            case 'z':
                cmd_vel.angular.z =  MAX_ANGULAR_VEL;
                break;
            case 'x':
                cmd_vel.angular.z = MIN_ANGULAR_VEL;
                break;
                // linear.xは前後方向の並進速度(m/s)。前方向が正。
                // angular.zは回転速度(rad/s)。反時計回りが正。
            }

        }

        pub.publish(cmd_vel);    // 速度指令メッセージをパブリッシュ（送信）
        ros::spinOnce();     // １回だけコールバック関数を呼び出す
        cmd_vel.linear.x  = 0.0; // 並進速度の初期化
        cmd_vel.linear.y  = 0.0; // 並進速度の初期化
        cmd_vel.angular.z = 0.0; // 回転速度の初期化
        rate.sleep();
    }

    return 0;
}

