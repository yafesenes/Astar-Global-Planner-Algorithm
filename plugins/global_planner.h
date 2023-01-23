#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector> 

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner {

    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
        public:

	int height;
	int width;
	bool initialized_;
	bool *occupancyGridMap;
	int mapSize;
	double resolution;
	double OriginX;
	double OriginY;
	int sayac = 0;	
	
	costmap_2d::Costmap2DROS *costmap_ros_;
	costmap_2d::Costmap2D *costmap_;        
	ros::NodeHandle ROSNodeHandle;
		
    GlobalPlanner();
    GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    double G_cost(unsigned int ParentX, unsigned int ParentY, double ParentGcost, unsigned int CurrentX, unsigned int CurrentY);
    double H_cost(unsigned int FinishX, unsigned int FinishY, unsigned int CurrentX, unsigned int CurrentY);
    double F_cost(double F_cost, double G_cost);
    int GridLocationX(double wx, double resolution);
    int GridLocationY(double wy, double resolution);
    void Print1DVector(std::vector<double> &Dim1Vect);
    void Print2DVector(std::vector<std::vector<double>> &Dim2Vect);
    void AddRowVectorTo2DVect(std::vector<double> &RowVector,std::vector<std::vector<double>> &Matrix);
    void GetColonSetVector(std::vector<double> &SetVector, std::vector<std::vector<double>> Matrix,int indexColumn);
    int GetIndexMinValueVector(std::vector<double> &Vector);
    void InitializeNeighbours(std::vector<double> &Vector, std::vector<std::vector<double>> &Matrix, int yMax, int xMax);
    void RemoveRowFrom2DVect(std::vector<std::vector<double>> &Matrix, int rowIndex);
	void GetRowSetVector(std::vector<double> &SetVector, std::vector<std::vector<double>> Matrix,int indexRow);
	bool IsTraversable(double X, double Y, bool *occupancyGridMap);
	bool IsInList(double xCoordinate, double yCoordinate, std::vector<std::vector<double>> &Matrix);
	void ConstructPath(std::vector<std::vector<double>> &ClosedList, std::vector<double> &TargetNode, std::vector<std::vector<double>> &BestPath, int StartX, int StartY);
	void Double2Float(std::vector<std::vector<double>> &DoubleVect, std::vector<std::vector<float>> &FloatVect, std::vector<float> &TempHelper);
	void floatPrint2DVector(std::vector<std::vector<float>> &Dim2Vect);
    };
};
#endif
