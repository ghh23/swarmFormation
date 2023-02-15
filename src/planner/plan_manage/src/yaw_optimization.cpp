#include <string.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/StdVector>
#include <iostream>

#include <iostream>
#include <cmath>
#include <cstring>
#include "tf/transform_broadcaster.h"

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include "vector"
#include <unsupported/Eigen/CXX11/Tensor>
#include "std_msgs/Float64.h" //普通文本类型的消息
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <quadrotor_msgs/yawOptimization.h>

#include <Eigen/Eigen>  
#include <stdlib.h>  
#include <Eigen/Geometry> 

#include <iostream>  
#include <Eigen/Eigen>  
#include <stdlib.h>  
#include <Eigen/Geometry>  
#include <Eigen/Core>  
#include <vector>  
#include <math.h>  
  

#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <random>
#include <ctime>
#include<eigen3/Eigen/Dense>



#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <random>
#include <ctime>
#include<eigen3/Eigen/Dense>
#include <bits/stdc++.h>
#define N  9 //精度为小数点后面2位


using namespace Eigen;
using namespace std;

const int dim = 7; // 6维，一共有6架飞机
const int p_num = 50; // 粒子数量
const int iters = 100; // 迭代次数
const int inf = 99999; //极大值
const double pi = 3.1415; 




//定义粒子的位置和速度的范围
//设置每一个维度的位置和速度的极大值和极小值
// Matrix<double, dim, 1>v_max;
// Matrix<double, dim, 1>v_min;

// Matrix<double, dim, 1>pos_max;
// Matrix<double, dim, 1>pos_min;

//这里先单独设置每一维都在相同的区间中
const double v_max = 4;
const double v_min = -4;
const double pos_max = 3.14;
const double pos_min = -3.14;

//定义位置向量和速度向量
Matrix<double, dim, p_num> pos;
Matrix<double, dim, p_num> speed;

//定义粒子的最优位置和全局最优位置
Matrix<double, dim, p_num> p_best;
Matrix<double, dim, 1> g_best;
Matrix<double, dim, 1> last_g_best;

//定义粒子的个体历史最佳适应度和种群最佳适应度
Matrix<double, 1, p_num>fitness_gbest;
double fitness_zbest;

//临时存放得出的最佳适应度
Matrix<double, 1, p_num>temp;
double yawTest[7] = {0};
double expectedYaw[7] = {0};



ros::Publisher fov_pub_;
ros::Publisher fov_edge_Pub;
ros::Publisher yawOptimization_pub;
ros::Publisher drone_frame_pub;
ros::Timer map_vis_timer_;
ros::Timer swarm_optimization_timer_;
tf::TransformBroadcaster* broadcaster;

//初始化fov_shape节点
double max_dis_ = 4.0;
double x_max_dis_gain_ = 0.64;
double y_max_dis_gain_ = 0.82;
double x_max_dis = max_dis_ * x_max_dis_gain_;
double y_max_dis = max_dis_ * y_max_dis_gain_;
std::vector<Eigen::Vector3d> fov_shape;
std::vector<Eigen::Vector3f> cur_position;
const double eps = 1e-6;

//定义三维张量，存储点坐标
Eigen::Tensor<double, 3> point_temp(7, 3, 3);


Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)  
{  
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());  
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());  
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());  
  
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;  
      
    return q;  
}

void fov_visual_init() {
  double x_max_dis = max_dis_ * x_max_dis_gain_;
  double y_max_dis = max_dis_ * y_max_dis_gain_;

  cur_position.resize(7);
  fov_shape.resize(3);
  fov_shape[0][0] = 0;
  fov_shape[0][1] = 0;
  fov_shape[0][2] = 0;

  fov_shape[1][2] = x_max_dis;
  fov_shape[1][1] = y_max_dis;
  fov_shape[1][0] = max_dis_;

  fov_shape[2][2] = x_max_dis;
  fov_shape[2][1] = -y_max_dis;
  fov_shape[2][0] = max_dis_;

}
//markerNode_fov.header.frame_id = "world";
string _frame_id;
//输入6台飞机的yaw角位置  得出三个点的坐标  输入计算



void drone_0_attitude_callback(const quadrotor_msgs::yawOptimization::ConstPtr& msg){

  cur_position[0][0] = msg->x;
  cur_position[0][1] = msg->y;
  cur_position[0][2] = msg->z;
  expectedYaw[0] = msg->expectedYaw;

}

void drone_1_attitude_callback(const quadrotor_msgs::yawOptimization::ConstPtr& msg){
  cur_position[1][0] = msg->x;
  cur_position[1][1] = msg->y;
  cur_position[1][2] = msg->z;
  expectedYaw[1] = msg->expectedYaw;
  
}

void drone_2_attitude_callback(const quadrotor_msgs::yawOptimization::ConstPtr& msg){
  cur_position[2][0] = msg->x;
  cur_position[2][1] = msg->y;
  cur_position[2][2] = msg->z;
  expectedYaw[2] = msg->expectedYaw;
}

void drone_3_attitude_callback(const quadrotor_msgs::yawOptimization::ConstPtr& msg){
  cur_position[3][0] = msg->x;
  cur_position[3][1] = msg->y;
  cur_position[3][2] = msg->z;
 expectedYaw[3] = msg->expectedYaw;
}

void drone_4_attitude_callback(const quadrotor_msgs::yawOptimization::ConstPtr& msg){
  cur_position[4][0] = msg->x;
  cur_position[4][1] = msg->y;
  cur_position[4][2] = msg->z;
  expectedYaw[4] = msg->expectedYaw;
}

void drone_5_attitude_callback(const quadrotor_msgs::yawOptimization::ConstPtr& msg){
  cur_position[5][0] = msg->x;
  cur_position[5][1] = msg->y;
  cur_position[5][2] = msg->z;
  expectedYaw[5] = msg->expectedYaw;
}

void drone_6_attitude_callback(const quadrotor_msgs::yawOptimization::ConstPtr& msg){
  cur_position[6][0] = msg->x;
  cur_position[6][1] = msg->y;
  cur_position[6][2] = msg->z;
  expectedYaw[6] = msg->expectedYaw;
}

//计算重叠三角形的面积/////////////////////////////////////////////////

int dcmp(double x)
{
    if(x > eps) return 1;
    return x < -eps ? -1 : 0;
}

struct Point
{
    double x, y;
};

double cross(Point a, Point b, Point c)  
{
    return (a.x-c.x)*(b.y-c.y)-(b.x-c.x)*(a.y-c.y);  //叉积公式
}

//计算线段ab和cd的交点坐标
Point intersection(Point a, Point b, Point c, Point d)
{
    Point p = a;
    double t =((a.x-c.x)*(c.y-d.y)-(a.y-c.y)*(c.x-d.x))/((a.x-b.x)*(c.y-d.y)-(a.y-b.y)*(c.x-d.x)); 
    p.x +=(b.x-a.x)*t; 
    p.y +=(b.y-a.y)*t;
    return p;
}

//计算多边形面积，将多边形拆解成连续三个顶点组合成的多个三角形进行计算，这个循环计算一次其实是计算两次多边形的面积。
double PolygonArea(Point p[], int n)
{
    if(n < 3) return 0.0;
    double s = p[0].y * (p[n - 1].x - p[1].x);
    for(int i = 1; i < n - 1; ++ i) {
        s += p[i].y * (p[i - 1].x - p[i + 1].x);
        // cout << "p[i-1].x =" << p[i-1].x << ", p[i-1].y=" << p[i-1].y << endl;
        // cout << "p[i].x =" << p[i].x << ", p[i].y=" << p[i].y << endl;
        // cout << "p[i+1].x =" << p[i+1].x << ", p[i+1].y=" << p[i+1].y << endl;
    }
    s += p[n - 1].y * (p[n - 2].x - p[0].x);
   
    return fabs(s * 0.5);
}

double CPIA(Point a[], Point b[], int na, int nb)  //ConvexPolygonIntersectArea
{
    Point p[20], tmp[20];
    int tn, sflag, eflag;
    memcpy(p,b,sizeof(Point)*(nb));
    for(int i = 0; i < na && nb > 2; i++)
    {
    	if (i == na - 1) {
    		sflag = dcmp(cross(a[0], p[0],a[i]));
    	} else {
    		sflag = dcmp(cross(a[i + 1], p[0],a[i]));
    	}
        for(int j = tn = 0; j < nb; j++, sflag = eflag)
        {
            if(sflag>=0) {
            	tmp[tn++] = p[j];
            }
            if (i == na - 1) {
            	if (j == nb -1) {
            		eflag = dcmp(cross(a[0], p[0], a[i]));
				} else {
					eflag = dcmp(cross(a[0], p[j + 1], a[i])); //计算下一个连续点在矢量线段的位置
				}
			} else {
				if (j == nb -1) {
					eflag = dcmp(cross(a[i + 1], p[0], a[i]));
				} else {
					eflag = dcmp(cross(a[i + 1], p[j + 1], a[i]));
				}
			}
            if((sflag ^ eflag) == -2) {  //1和-1的异或为-2，也就是两个点分别在矢量线段的两侧
            	if (i == na - 1) {
            		if (j == nb -1) {
            			tmp[tn++] = intersection(a[i], a[0], p[j], p[0]); //求交点
            		} else {
            			tmp[tn++] = intersection(a[i], a[0], p[j], p[j + 1]);
            		}
				} else {
					if (j == nb -1) {
						tmp[tn++] = intersection(a[i], a[i + 1], p[j], p[0]);
					} else {
						tmp[tn++] = intersection(a[i], a[i + 1], p[j], p[j + 1]);
					}
				}
            }
        }
        memcpy(p, tmp, sizeof(Point) * tn);
        nb = tn, p[nb] = p[0];
    }
    if(nb < 3) return 0.0;
    return PolygonArea(p, nb);
}

double SPIA(Point a[], Point b[], int na, int nb)    //SimplePolygonIntersectArea 调用此函数
{
    int i, j;
    Point t1[na], t2[nb];
    double res = 0, num1, num2;
    t1[0] = a[0], t2[0] = b[0];
    for(i = 2; i < na; i++)
    {
        t1[1] = a[i-1], t1[2] = a[i];
        num1 = dcmp(cross(t1[1], t1[2],t1[0]));  //根据差积公式来计算t1[2]在矢量线段（t1[0], t1[1]）的左侧还是右侧，
                                                 //值为负数在矢量线段左侧，值为正数在矢量线段右侧
        if(num1 < 0) swap(t1[1], t1[2]);  // 按逆时针进行排序
        for(j = 2; j < nb; j++)
        {
            t2[1] = b[j - 1], t2[2] = b[j];
            num2 = dcmp(cross(t2[1], t2[2],t2[0]));
            if(num2 < 0) swap(t2[1], t2[2]);  
            res += CPIA(t1, t2, 3, 3) * num1 * num2; 
        }
    }
    return res;
}

double calculate_overlapping_area(double *yawTest)
{
  double sum_overlap_area = 0; 
  for(int i = 0; i < 7; i++)
  {
    Eigen::Vector3d p(cur_position[i][0], cur_position[i][1], cur_position[i][2]);
    Eigen::Quaterniond q = euler2Quaternion(0,0,yawTest[i]);
    Eigen::Quaterniond q0(q.w(), q.x(), q.y(), q.z());


    for (int j = 0; j < (int)fov_shape.size(); j++) {
      Eigen::Vector3d vector_temp;
      vector_temp = q0 * fov_shape[j] + p;
      point_temp(i,j,0) = vector_temp[0];
      point_temp(i,j,1) = vector_temp[1];
      point_temp(i,j,2) = vector_temp[2];
 
    }
  }
    for(int i = 1; i < 3; i++){
        Point p1[3],p2[3];
        p1[0].x = point_temp(i,0,0);
        p1[0].y =  point_temp(i,0,1);

        p1[1].x = point_temp(i,1,0);
        p1[1].y = point_temp(i,1,1);

        p1[2].x = point_temp(i,2,0);
        p1[2].y = point_temp(i,2,1);
        
        for(int j = i; j< 3; j++){
            p2[0].x = point_temp(j,0,0);
            p2[0].y =  point_temp(j,0,1);

            p2[1].x = point_temp(j,1,0);
            p2[1].y = point_temp(j,1,1);

            p2[2].x = point_temp(j,2,0);
            p2[2].y = point_temp(j,2,1);
            double Area = SPIA(p1, p2, 3, 3); 
            if(Area > 0)
            {
                sum_overlap_area += Area;
            }

        }
        
    }
    //ROS_INFO("sum_overlap_area = %f", sum_overlap_area);
    return sum_overlap_area;
}

double calculate_fv(double *yawTest)
{
  double sum = 0.0;
  for(int i = 0; i < 7; i++)
  {
    
    sum +=  pow((yawTest[i] - expectedYaw[i]), 2);
  }
  return sum;
}


//定义适应度函数 
double calculate_sum( Matrix<double, 7, 1> pos)
{
  double fs = 0;
  double fd = 0;
  for(int i = 0; i < 7; i++)
  {
    yawTest[i] = pos(i,0);
    fs += pow((yawTest[i] - last_g_best(i,0)) , 2);
    
  }
  for(int i = 0; i < 7; i++)
  {
    for(int j = i; j < 7; j++)
    {
      fd += pow((yawTest[i] - yawTest[j]) , 2);
    }
  }
  //各项系数
  double a1 = 0.01 ,a2 = 0.1, a3 = 0.1, a4 = 0.01;
  //double a1 = 0.1 ,a2 = 0, a3 = 0, a4 = 0;
  double fol = calculate_overlapping_area(yawTest);
  double fv = calculate_fv(yawTest);
  double J = a1 * fol + a2 * fv  + a3 * fs - a4 * fd;  
  return J;
}

void init()
{
    srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
    //生成初始化种群，首先随机生成初始种群位置
    for(int i = 0; i < dim; i++)
    {
        for(int j = 0; j < p_num; j++)
        {
            pos(i,j) = pos_min + (rand() % (N+1) /(float)(N+1))*(pos_max-pos_min);
            speed(i, j) = v_min + (rand() % (N+1) /(float)(N+1))*(v_max-v_min);
        }
       
    }
 
    //每个个体的历史最佳位置
    p_best = pos;
    //种群的历史最佳位置
    g_best = pos.col(0);
    
    
    //每个个体的历史最佳适应度
    for(int i = 0; i <p_num; i++) 
    {  
        fitness_gbest(0,i) = calculate_sum(pos.col(i));
        
    }
    //种群的历史最佳适应度
   fitness_zbest = fitness_gbest(0);

   //更新种群的最佳适应度
   for(int i = 0 ; i< p_num; i++)
   {
    if(fitness_zbest > fitness_gbest(i))
    {
      fitness_zbest = fitness_gbest(i);
      g_best = pos.col(i);
    }
   }

}


void PSO()
{
    //产生随机数
    init();
    srand(time(NULL));//设置随机数种子，使每次产生的随机序列不同
   
    for(int step = 1; step < iters; ++step)
    {
        for(int i = 0; i < p_num; i++)
        {
            //更新速度向量并对速度进行边界处理
           // speed.col(i) = 0.5 * speed.col(i) + 2* (rand() % (N+1) /(float)(N+1)) * (p_best.col(i) - pos.col(i)) + 2 *(rand() % (N+1) /(float)(N+1))* (g_best - pos.col(i));
            //更新位置并对位置进行边界处理
            // pos.col(i) = pos.col(i) + speed.col(i);
            for(int j = 0; j < dim; j++)
            {
                speed(j,i) = 0.5 * speed(j,i) + 0.8* (rand() % (N+1) /(float)(N+1)) * (p_best(j,i) - pos(j,i)) + 0.3 *(rand() % (N+1) /(float)(N+1))* (g_best(j,0) - pos(j,i));
            
                if (speed(j,i) < -2)
                   speed(j,i) = -2;
                if (speed(j,i) > 2)
                    speed(j,i) = 2;

                //更新位置并对位置进行边界处理
                pos(j,i) = pos(j,i) + speed(j,i);
                if (pos(j,i) < -3.14)
                    pos(j,i) = -3.14;
                if (pos(j,i) > 3.14)
                    pos(j,i) = 3.14;
            }
            // 自适应变异
            // for(int j = 0 ; j < dim; j++)
            // {
            //   if ((rand() % (N+1) /(float)(N+1)) > 0.85)
            //   pos(j,i) = pos_min + (pos_max-pos_min) * (rand() % (N+1) /(float)(N+1));

            // }



           // 计算新种群各个个体位置的适应度
           temp(0,i) = calculate_sum(pos.col(i));

            //新适应度与个体历史最佳适应度作比较
            if(temp(0,i) < fitness_gbest(0,i))
            {
                //更新个体历史最佳适应度
                fitness_gbest(0,i) = temp(0,i);
                //更新每个个体历史最佳位置
                p_best.col(i) = pos.col(i);
            }

            //个体历史最佳适应度与种群历史最佳适应度做比较
            if(fitness_gbest(0,i) < fitness_zbest )
            {
                //更新群体历史最佳适应度
                fitness_zbest = fitness_gbest(0,i);
                //更新群体历史最佳位置
                g_best = p_best.col(i);
              
            }
            

        }

   
    }
   
   
}



void swarmOptimizationCallback(const ros::TimerEvent& event)
{

  quadrotor_msgs::yawOptimization yawCal;
  
  PSO();

  for(int i = 0; i < 7; i++)
  {
    yawCal.calculateYaw[i] = g_best(i,0);
 
    last_g_best(i,0) = g_best(i,0);
  }
  
  yawOptimization_pub.publish(yawCal);
  
  
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "yaw_optimization");
    ros::NodeHandle nh("~");
    fov_visual_init();
    ros::Subscriber yawExpected_sub_0 = nh.subscribe("/drone_0_planning/expectedYaw", 100,  drone_0_attitude_callback);
    ros::Subscriber yawExpected_sub_1 = nh.subscribe("/drone_1_planning/expectedYaw", 100,  drone_1_attitude_callback);
    ros::Subscriber yawExpected_sub_2 = nh.subscribe("/drone_2_planning/expectedYaw", 100,  drone_2_attitude_callback);
    ros::Subscriber yawExpected_sub_3 = nh.subscribe("/drone_3_planning/expectedYaw", 100,  drone_3_attitude_callback);
    ros::Subscriber yawExpected_sub_4 = nh.subscribe("/drone_4_planning/expectedYaw", 100,  drone_4_attitude_callback);
    ros::Subscriber yawExpected_sub_5 = nh.subscribe("/drone_5_planning/expectedYaw", 100,  drone_5_attitude_callback);
    ros::Subscriber yawExpected_sub_6 = nh.subscribe("/drone_6_planning/expectedYaw", 100,  drone_6_attitude_callback);

  
    yawOptimization_pub = nh.advertise<quadrotor_msgs::yawOptimization>("/yawCalculate", 10);
    swarm_optimization_timer_ = nh.createTimer(ros::Duration(0.01), swarmOptimizationCallback);
   

    
    tf::TransformBroadcaster b;
    broadcaster = &b;

    ros::spin();

    return 0;
}
