{
      			for (unsigned int ix = 0; ix < width; ix++)
      			{
        			unsigned int cost = static_cast<int>((costmap_ros->getCostmap())->getCost(ix, iy));

        			if (cost < 150)
          				occupancyGridMap[iy * width + ix] = true;
        			else
          				occupancyGridMap[iy * width + ix] = false;
      			}
    		}