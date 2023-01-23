#include <pluginlib/class_list_macros.h>
#include "global_planner.h"
#include <pluginlib/class_loader.h>
#include <math.h>
#include <cmath>
#include <cfenv>
#include <climits>
#include <tf/tf.h>

/* Düzeltilmesi gerekenler
1- Costmap doğru mu? Traverse fonksiyonunu düzenle DÜZELTİLDİ
2- Target Node a ulaşıldığına return mekanizması yapılmalı
3- While döngüsü kırılmalı!
*/

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

int sayac = 0;
int ctrTargetGoalX=0;
int ctrTargetGoalY=0;
int ctrTargetCalculatedX=1;
int ctrTargetCalculatedY=1;

namespace global_planner {
    GlobalPlanner::GlobalPlanner ()
    {

	}
	
    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
    		initialize(name, costmap_ros);
	}

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {     		        	
		/*Haritanın her gridine özel değişken oluşturulması*/
		height = (costmap_ros->getCostmap())->getSizeInCellsY();
		width = (costmap_ros->getCostmap())->getSizeInCellsX();    		
		mapSize = width * height;   		
    	occupancyGridMap = new bool[mapSize];

		/*Global costmap'e göre occupancy değerlerinin belirlenmesi*/
		for (unsigned int iy = 0; iy < height; iy++)
		{
			for (unsigned int ix = 0; ix < width; ix++)
			{
				unsigned int cost = static_cast<int>((costmap_ros->getCostmap())->getCost(ix, iy));

				if (cost < 150)		/*Threshold değeridir, engellere daha yakın hareket etmesine izin vermek için yükseltilebilir*/
					occupancyGridMap[iy * width + ix] = true;	/*Robot bu gridlerde hareket edebilir*/
				else
					occupancyGridMap[iy * width + ix] = false;	/*Robot hareket edemez*/
			}
		}
    		
		/*Global cost map in bastırılması için test kodu*/
		/*Occupancy threshold değerinin test edilmesi için kullanıldı*/
		/*
		for (unsigned int iy = 120; iy < 260; iy++)
		{
			for (unsigned int ix = 150; ix < 270; ix++)
			{			
				std::cout<<occupancyGridMap[iy * width + ix];			
				unsigned int cost = static_cast<int>((costmap_ros->getCostmap())->getCost(ix, iy));
				std::cout<<cost;
				std::cout<<" ";
			}
			std::cout<<"\n";
		}
		*/
		
		ROS_INFO("#######################################");		
		ROS_INFO("Width= %d",width);
		ROS_INFO("Height= %d",height);
		ROS_INFO("GridSize= %d",width*height);		
		
		OriginX = (costmap_ros->getCostmap())->getOriginX();
		ROS_INFO("OriginX= %f",OriginX);
		
		OriginY = (costmap_ros->getCostmap())->getOriginY();
		ROS_INFO("OriginY= %f",OriginY);
		
		resolution = (costmap_ros->getCostmap())->getResolution();
		ROS_INFO("Resolution= %f",resolution);
		
		double d = (costmap_ros->getCostmap())->getSizeInMetersX();
		ROS_INFO("MetersX= %f",d);
		
		double e = (costmap_ros->getCostmap())->getSizeInMetersY();
		ROS_INFO("MetersY= %f",e);	        	
	}        
        
	bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
	{
		int positionX = 0;
		int positionY = 1;
		int positionParentX=2;
		int positionParentY=3;
		int positionParentGcost=4;
		int positionGcost= 5;
		int positionFcost= 6;
		
		int BestPathSizeX;
		int BestPathSizeY; 	
		
		unsigned int StartX_grid;
		unsigned int StartY_grid;
		unsigned int FinishX_grid;
		unsigned int FinishY_grid;
		bool flag=1;
		int TempFcostIndex;
		double TempHcost;
		double TempGcost;
		double TempFcost;
										
		std::vector<std::vector<double>> OpenList;
		std::vector<std::vector<double>> ClosedList;
		std::vector<std::vector<double>> TempNeighbourList;
		std::vector<std::vector<double>> BestPath;
		std::vector<double> CurrentNode;
		std::vector<unsigned int> TargetNode;
		std::vector<double> TempFcostVect;
		std::vector<double> TempNeighbourVect; 
		std::vector<float> TempHelper;       	      
		
		std::vector<std::vector<float>> FloatBestPath {{0,0}};
		
		StartX_grid= GridLocationX(start.pose.position.x,resolution);
		StartY_grid= GridLocationY(start.pose.position.y,resolution);
		FinishX_grid= GridLocationX(goal.pose.position.x,resolution);
		FinishY_grid= GridLocationY(goal.pose.position.y,resolution);
		
		TargetNode= {FinishX_grid, FinishY_grid};       
		
		std::vector<double> StartNode
		{
			double(StartX_grid),	/*X location of start node*/
			double(StartY_grid),	/*Y location of start node*/
			double(StartX_grid),	/*ParentX!*/
			double(StartY_grid),	/*ParentY!*/
			0,			/*ParentGcost*/
			0,			/*Gcost*/
			F_cost(H_cost(FinishX_grid, FinishY_grid, StartX_grid, StartY_grid),0)/*Calculate F cost*/
		};		
	
		AddRowVectorTo2DVect(StartNode,OpenList);	/*Başlangıç noktasının açık listeye eklenmesi*/

		ROS_INFO("kac kere geldim?");
		ROS_INFO("%d",sayac);
		
		ctrTargetGoalX=FinishX_grid;
		ctrTargetGoalY=FinishY_grid;

		while(ctrTargetGoalX != ctrTargetCalculatedX || ctrTargetGoalY != ctrTargetCalculatedY)
		{
			ROS_INFO("iceri");
			/*Determine Current Node*/			
			GetColonSetVector(TempFcostVect, OpenList, positionFcost);			
			TempFcostIndex=GetIndexMinValueVector(TempFcostVect);
			GetRowSetVector(CurrentNode, OpenList, TempFcostIndex);			
			RemoveRowFrom2DVect(OpenList, TempFcostIndex);
			AddRowVectorTo2DVect(CurrentNode, ClosedList);	
			
			/*Rotanın hedefe ulaşıp ulaşmadığı kontrol edilir*/
			if(CurrentNode[positionX]==TargetNode[positionX] && CurrentNode[positionY]==TargetNode[positionY])
			{
				/*sayac = 300;*/
				
				ConstructPath(ClosedList, CurrentNode, BestPath, StartX_grid, StartY_grid);
				ROS_INFO("best path x-y coordinates");
				Print2DVector(BestPath);
				
				BestPathSizeY = BestPath.size();
				ROS_INFO("number of coordinates: %d", BestPathSizeY);
				
				Double2Float(BestPath, FloatBestPath, TempHelper);
										
				geometry_msgs::PoseStamped road = goal;
				tf::Vector3 vectorToTarget;
					
				/*	
				std::cout<<goal.pose.position.x<<std::endl;
				std::cout<<goal.pose.position.y<<std::endl;*/
					
				plan.push_back(start);       			       			
				
				for (int i = 0; i < BestPath.size(); i++)
				{
					road.pose.position.x = FloatBestPath[BestPathSizeY-i][0];
					road.pose.position.y =  FloatBestPath[BestPathSizeY-i][1];
					road.pose.position.z = 0.0;
					/*road.pose.orientation = tf::createQuaternionMsgFromYaw(90);*/   					  					
					plan.push_back(road);	
							        	
				}
				
				plan.push_back(goal);

				ctrTargetCalculatedX=ctrTargetGoalX;
				ctrTargetCalculatedY=ctrTargetGoalY;				
			}
				
			InitializeNeighbours(CurrentNode, TempNeighbourList, height, width);
			
			for (int i = 0; i < TempNeighbourList.size(); i++)
			{   			
				/*Robot ilgili gride gidebilir mi? veya İlgili grid daha önceden incelenmiş miydi?*/
				if(IsTraversable(TempNeighbourList[i][0],TempNeighbourList[i][1],occupancyGridMap)==0 || IsInList(TempNeighbourList[i][0],TempNeighbourList[i][1], ClosedList))
				{
					continue;
				}
				
				if(IsInList(TempNeighbourList[i][0],TempNeighbourList[i][1], OpenList))
				{			
					TempGcost= G_cost(TempNeighbourList[i][2], TempNeighbourList[i][3], TempNeighbourList[i][4], TempNeighbourList[i][0], TempNeighbourList[i][1]);
					TempHcost= H_cost(FinishX_grid, FinishY_grid, TempNeighbourList[i][0], TempNeighbourList[i][1]);
					TempFcost= TempHcost + TempGcost;
					
					if(TempFcost < TempNeighbourList[i][6])
					{
						TempNeighbourList[i][2]= CurrentNode[0];
						TempNeighbourList[i][3]= CurrentNode[1];
						TempNeighbourList[i][4]= CurrentNode[5];
						TempNeighbourList[i][5]= TempGcost;
						TempNeighbourList[i][6]= TempFcost;        					
					}    										
				}
				
				else
				{
					TempGcost= G_cost(TempNeighbourList[i][2], TempNeighbourList[i][3], TempNeighbourList[i][4], TempNeighbourList[i][0], TempNeighbourList[i][1]);
					TempHcost= H_cost(FinishX_grid, FinishY_grid, TempNeighbourList[i][0], TempNeighbourList[i][1]);
					TempFcost= TempHcost + TempGcost;
					
					TempNeighbourList[i][2]= CurrentNode[0];
					TempNeighbourList[i][3]= CurrentNode[1];
					TempNeighbourList[i][4]= CurrentNode[5];
					TempNeighbourList[i][5]= TempGcost;
					TempNeighbourList[i][6]= TempFcost;    
					
					TempNeighbourVect={
						TempNeighbourList[i][0],
						TempNeighbourList[i][1],        					
						TempNeighbourList[i][2],
						TempNeighbourList[i][3],
						TempNeighbourList[i][4],
						TempNeighbourList[i][5],
						TempNeighbourList[i][6]   				
					};
					

					AddRowVectorTo2DVect(TempNeighbourVect,OpenList);
				} 
			}
					
			sayac = sayac + 1;    					
		}

		return true;
	}
        
	/*normal koordinatın grid map'e göre konumunun belirlenmesini sağlar*/   
	int GlobalPlanner::GridLocationX(double wx, double resolution)
	{        	
		int mx;        	
		mx = (int) ((wx - OriginX) / resolution);	  	
		return mx;
	}
	
	/*normal koordinatın grid map'e göre konumunun belirlenmesini sağlar*/   
	int GlobalPlanner::GridLocationY(double wy, double resolution)
	{        	
		int my;        	
		my = (int) ((wy - OriginY) / resolution);	  	
		return my;
	}
	
	/*Parent G_cost'a parent node ile arasındaki mesafeyi ekleyerek G cost u hesaplar*/
	double GlobalPlanner::G_cost(unsigned int ParentX, unsigned int ParentY, double ParentGcost, unsigned int CurrentX, unsigned int CurrentY)
	{
		return ParentGcost+sqrt((CurrentY-ParentY)*(CurrentY-ParentY)+(CurrentX-ParentX)*(CurrentX-ParentX));
	}
	
	/*Bitiş noktasıyla arasındaki uzaklığı hesaplamaya yarar*/
	double GlobalPlanner::H_cost(unsigned int FinishX, unsigned int FinishY, unsigned int CurrentX, unsigned int CurrentY)
	{
		return sqrt((CurrentY-FinishY)*(CurrentY-FinishY)+(CurrentX-FinishX)*(CurrentX-FinishX));
	}
	
	/*Gcost ve Hcost'un toplamını verir.*/
	double GlobalPlanner::F_cost(double H_cost, double G_cost)
	{
		return H_cost + G_cost;
	}
	
	void GlobalPlanner::Print1DVector(std::vector<double> &Dim1Vect)
	{
			for (int j = 0; j < Dim1Vect.size(); j++)
			{
				std::cout << Dim1Vect[j] << " ";
			} 
	}

	void GlobalPlanner::Print2DVector(std::vector<std::vector<double>> &Dim2Vect)
	{
    		for (int i = 0; i < Dim2Vect.size(); i++)
    		{
        		for (int j = 0; j < Dim2Vect[i].size(); j++)
        		{
            			std::cout << Dim2Vect[i][j] << " ";
        		}    
        		std::cout << std::endl;
		}
	}
	
	void GlobalPlanner::floatPrint2DVector(std::vector<std::vector<float>> &Dim2Vect)
	{
    		for (int i = 0; i < Dim2Vect.size(); i++)
    		{
        		for (int j = 0; j < Dim2Vect[i].size(); j++)
        		{
            			std::cout << Dim2Vect[i][j] << " ";
        		}    
        		std::cout << std::endl;
		}
	}        
        
	/*2d vektörün istenilen sütünunu 1d vektörün içine koyar*/
	void GlobalPlanner::GetColonSetVector(std::vector<double> &SetVector, std::vector<std::vector<double>> Matrix,int indexColumn)
	{
    		SetVector.clear();
    
    		for (int i = 0; i < Matrix.size(); i++)
    		{
        		for (int j = 0; j < Matrix[i].size(); j++)
        		{
					if(indexColumn==j)
	               	{
						SetVector.push_back(Matrix[i][j]);
					}
            
        		}    
		}
	}

	/*2d vektörün istenilen satırı 1d vektörün içine koyar*/
	void GlobalPlanner::GetRowSetVector(std::vector<double> &SetVector, std::vector<std::vector<double>> Matrix,int indexRow)
	{
		SetVector.clear();

		for (int i = 0; i < Matrix.size(); i++)
		{
			for (int j = 0; j < Matrix[i].size(); j++)
			{
					if(indexRow==i)
					{
						SetVector.push_back(Matrix[i][j]);
					}
		
			}    
		}
	}

	/*1d vektördeki en küçük elemanının indexini verir*/
	int GlobalPlanner::GetIndexMinValueVector(std::vector<double> &Vector)
	{
		return std::distance(std::begin(Vector), std::min_element(std::begin(Vector), std::end(Vector)));
	}

	/*2d vektöre verilen vektörü satır olarak ekler*/
	void GlobalPlanner::AddRowVectorTo2DVect(std::vector<double> &RowVector, std::vector<std::vector<double>> &Matrix)
	{
   		Matrix.push_back(RowVector);
	}

	/*2d vektör içinden verilen indexdeki satırı siler*/
	void GlobalPlanner::RemoveRowFrom2DVect(std::vector<std::vector<double>> &Matrix, int rowIndex)
	{
		Matrix.erase(Matrix.begin() + rowIndex);
	}

	/*Komşuları listeye ekler*/
	void GlobalPlanner::InitializeNeighbours(std::vector<double> &Vector, std::vector<std::vector<double>> &Matrix, int yMax, int xMax)
	{
		double distance;
	
		std::vector<std::vector<double>> Sum
		{
			{{1,0},{0,1},{-1,0},{0,-1},{1,1},{-1,1},{-1,-1},{1,-1}}
		};

		std::vector<double> TempVect;
		Matrix.clear();

		for (int j = 0; j < 8; j++)
		{
			distance= sqrt((Sum[j][0]*Sum[j][0])+(Sum[j][1]*Sum[j][1]));
			TempVect={Vector[0]+Sum[j][0], Vector[1]+Sum[j][1], Vector[0], Vector[1], Vector[5], Vector[5]+distance};
	
			if(TempVect[0]>xMax)
			{
					continue;
			}
	
			if(TempVect[1]>yMax)
			{
					continue;
			}
	
			AddRowVectorTo2DVect(TempVect, Matrix);
		} 
	}
	
	/*Verilen koordinattaki grid engel içeriyor mu?*/
	bool GlobalPlanner::IsTraversable(double X, double Y, bool *occupancyGridMap)
	{
		int index= X+Y*width;		
		return occupancyGridMap[index];
	}
	
	/*Verilen koordinatın listede olup olmadığını döndürür*/
	bool GlobalPlanner::IsInList(double xCoordinate, double yCoordinate, std::vector<std::vector<double>> &Matrix)
	{
    		for (int i = 0; i < Matrix.size(); i++)
    		{
        		if(Matrix[i][0]==xCoordinate && Matrix[i][1]==yCoordinate)
        		{
            			return 1;
        		}
		}
	
		return 0;
	}
	
	/*Target Node ulaşıldıktan sonra optimal path'i içeren 2d matrix oluşturur*/
	void GlobalPlanner::ConstructPath(std::vector<std::vector<double>> &ClosedList, std::vector<double> &TargetNode, std::vector<std::vector<double>> &BestPath, int StartX, int StartY)
	{
		bool flag=1;
		bool flag2=1;
		double wx;
		double wy;
		double researchX = TargetNode[2];
		double researchY = TargetNode[3];
		int i=0;
		
		std::vector<double> TempVect;
		std::vector<double> TempVect2;
		
		BestPath = {{TargetNode[0],TargetNode[1]},
			    {TargetNode[2],TargetNode[3]}};
			    
		while(flag)
		{			
			/*for (int i = 0; i < ClosedList.size(); i++)*/
			i=0;
			
			while(flag2)
    			{
    			
        			if(researchX==ClosedList[i][0] && researchY==ClosedList[i][1])
        			{
        				TempVect = {ClosedList[i][2],ClosedList[i][3]};
           				AddRowVectorTo2DVect(TempVect,BestPath);
					i=0;
        				researchX= TempVect[0];
        				researchY= TempVect[1]; 			       				        				
        			}
        			
        			if(researchX==StartX && researchY==StartY)
        			{
        				flag=0;
        				flag2=0;
        				break;
        			}
        			
        			i=i+1;
			}
		}

		for (int i = 0; i < BestPath.size(); i++)
		{
			
			wx = OriginX + (BestPath[i][0] + 0.5) * resolution;
	  		wy = OriginY + (BestPath[i][1] + 0.5) * resolution;
	  		
	  		BestPath[i][0]=wx;
	  		BestPath[i][1]=wy;	  		
		}
		
	}
	
	/*Double olarak hesaplanan optimum rotanın float olarak verilmesi gerektiği için dönüşüm fonksiyonu yazıldı*/
	void GlobalPlanner::Double2Float(std::vector<std::vector<double>> &DoubleVect, std::vector<std::vector<float>> &FloatVect, std::vector<float> &TempHelper)
	{
		float temp1;
		float temp2;

		for (int i = 0; i < DoubleVect.size(); i++)
		{
			temp1 = (float) DoubleVect[i][0];
			temp2 = (float) DoubleVect[i][1];
	
			TempHelper ={temp1, temp2}; 
			FloatVect.push_back(TempHelper);
		}
	} 

};
    




