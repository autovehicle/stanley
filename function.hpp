# include <iostream>
# include <string>
# include <fstream>
# include <iomanip>
# include <algorithm>
# include <ros/ros.h>
# include <morai_msgs/GPSMessage.h>
# include <sensor_msgs/Imu.h> //imu 데이터값 들어있는 메세지 헤더파일 include
# include <cmath>
# include <vector>
# include <ros/package.h>
# define _USE_MATH_DEFINES

using namespace std;

double pi = M_PI;
const double e = 0.006694379991;
const double a = 6378137.0;

double current_e = 0.0;
double current_n = 0.0;
double current_yaw = 1.0;

// 경로 저장 컨테이너
struct egoPath {double e, n ,u;};       // path.txt의 한줄을 struct에 저장
static vector<egoPath> path;            // 모든 경로 점을 이 배열에 저장(추후 인덱스 접근가능)


void gpsTf (const morai_msgs::GPSMessage::ConstPtr& msg, double& current_e, double& current_n) {
    // GPSMessage에서 받아올 WGS(GPS)값과 그 값에 대한 변수 설정
    double wgs_lat = msg->latitude;
    double wgs_lon = msg->longitude;
    double wgs_alt = msg->altitude;

    // WGS값 radian으로 변경(계산 목적)
    double rad_lat = wgs_lat * pi / 180;
    double rad_lon = wgs_lon * pi / 180;
    double rad_alt = wgs_lon * pi / 180;
    double k = a / sqrt(1-e*pow(sin(rad_lat), 2));

    // WGS->ECEF
    double ecef_x = k * cos(rad_lat) * cos(rad_lon);
    double ecef_y = k * cos(rad_lat) * sin(rad_lon);
    double ecef_z = k * (1-e) * sin(rad_lat);

    // 과제에서 주어진 reference ecef 값을 ref.txt 파일에서 불러와 새로운 변수에 저장
    double ref_lat;
    double ref_lon;
    double ref_alt;

    // 파일의 절대경로 찾기
    std::string package_path = ros::package::getPath("find_path");
    std::string file_path = package_path + "/ref.txt"; // (ref.txt는 패키지 폴더 최상단에 있어야 함)

    ifstream inputFile(file_path);
    
    if (inputFile.is_open()) {
        inputFile >> ref_lat;
        inputFile >> ref_lon;
        inputFile >> ref_alt;

        inputFile.close();
    }

    else {
        cerr << "fstream 오류입니다." << endl;
        return ;
    }

    double ref_rad_lat = ref_lat * pi / 180;
    double ref_rad_lon = ref_lon * pi / 180;
    double ref_rad_alt = ref_lon * pi / 180;
    double ref_k = a / sqrt(1-e*pow(sin(ref_rad_lat), 2));

    double ref_ecef_x = ref_k * cos(ref_rad_lat) * cos(ref_rad_lon);
    double ref_ecef_y = ref_k * cos(ref_rad_lat) * sin(ref_rad_lon);
    double ref_ecef_z = ref_k * (1-e) * sin(ref_rad_lat);


    // ecef 좌표 enu 좌표로 변환
    current_e = (-sin(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lon)*(ecef_y - ref_ecef_y));
    current_n = (-sin(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) - sin(rad_lat)*sin(rad_lon)*(ecef_y - ref_ecef_y) + cos(rad_lat)*(ecef_z - ref_ecef_z));
    double current_u = (cos(rad_lat)*cos(rad_lon)*(ecef_x - ref_ecef_x) + cos(rad_lat)*sin(rad_lon)*(ecef_y-ref_ecef_y) + sin(rad_lat)*(ecef_z-ref_ecef_z));

      


}

void getYaw (const sensor_msgs::Imu::ConstPtr& msg, double& current_yaw) {
    double imu_x = msg->orientation.x;
    double imu_y = msg->orientation.y;
    double imu_z = msg->orientation.z;
    double imu_w = msg->orientation.w;

    // imu 쿼터니안 -> 오일러 변환 (yaw 각 범위 -180 < yaw < 180)
    current_yaw = atan2(2 * (imu_w * imu_z + imu_x * imu_y), 1-2 * (pow(imu_y, 2) + pow(imu_z, 2)));
}

bool loadPath() {
    // ros::spin() 이 돌면서 매번 path.txt 파일을 열 것을 대비해 if문으로 egoPath에 숫자double ax, double ay, double cx, double cy, double e, double n, double u열이 들어있으면 다시 열지 말것
    if (!path.empty()) {
        return true;
    }

    // ros가 find_path 패키지의 절대경로를 찾음
    string pkg_path = ros::package::getPath("find_path");
    // 패키지 안에 파일이 들어있는 디렉토리를 언급해줌으로써 절대 경로 생성
    string file_path = pkg_path + "/path.txt";

    ifstream inputFile;
    inputFile.open(file_path);

    // file 열기 실패했을때의 if문 -> 디버깅 지점 확인 용도
    if (!inputFile.is_open()) {
        cout << "파일 열기에 실패 !" << endl;
        return false;
    }

    double e;
    double n;
    double u;
    while (inputFile >> e >> n >> u) {
        path.push_back(egoPath{e, n, u});
    }

    if (path.empty()) {
        cout << "egoPath가 비어있습니다 !" << endl;
        return false;
    }
    return true;
}

int findClosestPoint (const vector<egoPath>& path) {
    int closest_index = 0;
    double min_d = -1.0;

    for (int i = 0; i < path.size(); i++) {
        double de = path[i].e - current_e;
        double dn = path[i].n - current_n;
        double d = hypot(de, dn);

        if (min_d == -1.0 || d < min_d) {
            min_d = d;
            closest_index = i;
        }
    }
    return closest_index;
}

egoPath detectFront (const vector<egoPath>& path) {
    if (path.empty()) {
        cout <<"경로를 못 찾았습니다" << endl;
        return egoPath{current_e, current_n, 0.0};
    }

    // 1. 현재 차와 가장 가까운 경로상의 인덱스 찾기
    int closest_idx = findClosestPoint(path);

    // 2. 그 인덱스부터 "앞으로"만 경로를 탐색
    for (int i = closest_idx; i < path.size(); ++i) {
        double de = path[i].e - current_e;
        double dn = path[i].n - current_n;
        double d = hypot(de, dn);

        // 3. 첫 번째 점을 찾으면 반환
        if (d > 0) {
            return path[i];
        }
    }

    // 4. (경로 끝에 도달) 경로의 가장 마지막 점을 목표
    return path.back();
}

double getCurve (const vector<egoPath>& path) {
    
    int closest_idx = findClosestPoint(path);
    int next_idx = closest_idx + 1;

    double de = path[next_idx].e - path[closest_idx].e;
    double dn = path[next_idx].n - path[closest_idx].n;

    double curve = atan2(dn, de);

    return curve;
}

double getYawErr (const vector<egoPath>& path) {
    
    double path_yaw = getCurve(path);
    
    double yaw_err = path_yaw - current_yaw;

    // cout << "===========" << endl;
    // cout << setprecision(6) << endl;
    // cout << "Next Idx : " << next_idx << " \n"
    //     << " Clost Idx : " << closest_idx << endl;
    // cout << "DE : " << de << " DN : " << dn << endl;
    // cout << "Yaw : " << path_yaw << endl;

    // cout << path[next_idx].e << " ||| " << path[closest_idx].e << endl;
    // cout << path[next_idx].n << " ||| " << path[closest_idx].n << endl;
    // cout << path_yaw << endl;
    // cout << current_yaw << endl;
    // cout << yaw_err << endl;
    // cout << closest_idx << endl;
    // cout << "============" << endl;

    if(yaw_err > pi) yaw_err = yaw_err - 2 * pi;
    else if (yaw_err < - pi) yaw_err = yaw_err + 2 * pi;

    return yaw_err;
}

double getSignedDistanceErr(const vector<egoPath>& path, int closest_idx) {
    // 1. detect할 경로의 시작점(p_start)과 끝점(p_end)을 정의
    egoPath p_start, p_end;

    if (closest_idx >= path.size() - 1) {       // 경로 끝에 도달
        p_start = path[closest_idx - 1];
        p_end = path[closest_idx];
    } else {                                    // 경로 중간
        p_start = path[closest_idx + 1];
        p_end = path[closest_idx + 2];
    }

    // 2. 경로 벡터 (Vector A: p_start -> p_end) 계산
    double vecA_e = p_end.e - p_start.e;
    double vecA_n = p_end.n - p_start.n;

    // 3. 차량 벡터 (Vector B: p_start -> vehicle) 계산
    double vecB_e = p_start.e - current_e;
    double vecB_n = p_start.n - current_n;

    // 4. 외적 (A.e * B.n) - (A.n * B.e) 계산 : distance error의 부호를 결정하기 위함
    double cross_product_z = (vecA_e * vecB_n) - (vecA_n * vecB_e);

    // 5. 부호 결정 (왼쪽: +, 오른쪽: -)
    int sign = (cross_product_z > 0) ? 1 : -1;

    // 6. 오차의 크기 계산: 가장 가까운 점 (path[closest_index])까지의 절대 거리
    double de = path[closest_idx].e - current_e;
    double dn = path[closest_idx].n - current_n;
    double magnitude = hypot(de, dn);

    // 7. 부호 * 크기 반환
    return sign * magnitude;
}

int findConerIdx (const vector<egoPath>& path) {
    int conerIdx = 0;
    

    for (int i = 0; i < path.size(); i++) {
        double de = path[i].e - current_e;
        double dn = path[i].n - current_n;
        double curve = atan2(dn, de);

        if (curve > 0.5 || curve < -0.5) {
            conerIdx = i - 5;
        }
    }
    return conerIdx;
}