#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include "MyHeader.h"
#include "MyHeaderDefinitions.cpp"
#include <openrave/planningutils.h>

using namespace OpenRAVE;

class MyNewModule : public ModuleBase
{
public:
    MyNewModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommandRRT",boost::bind(&MyNewModule::MyCommandRRT,this,_1,_2),
                        "This is an example command");
        RegisterCommand("ShortCutSmoothing",boost::bind(&MyNewModule::ShortCutSmoothing,this,_1,_2),
				        "This is an example command wheeeee");

    }
    virtual ~MyNewModule() {}
    
    bool MyCommandRRT(std::ostream& sout, std::istream& sinput)
    {    	
        std::string input; 
        configurationType goalconfig, randConfig;    

        int count =0;
        float floatInput;
        while (count!=7) {
        	count++;
        	sinput>>floatInput;
        	goalconfig.push_back(floatInput);

        }
        // for (int i=0; i<7; i++)
	        // std::cout<<goalconfig[i]<<" ";
        // get angle limits for each joint, find rand point within limits
        OpenRAVE::EnvironmentBasePtr env = GetEnv();
        std::vector<RobotBasePtr> vbodies;
        env->GetRobots(vbodies);
        RobotBasePtr robot;
        robot = vbodies[0];
        std::vector<dReal> startConfig;
        robot->GetActiveDOFValues(startConfig);        
        std::vector<dReal> lower,upper;
	    robot->GetActiveDOFLimits(lower,upper); // lower and upper are 1D vectors of 7 elts
        // lower[4] = lower[6] = -M_PI;
        // upper[4] = upper[6] = M_PI;

        RRTNode *nearestNode = new RRTNode(startConfig);
		NodeTree tree((new RRTNode(startConfig)));
		// NodeTree *ptrTree = &tree;

		time_t start,now;
		time(&start);
		time(&now);
		int goalReachedFlag = 0;
		float distToGoal; 
				TrajectoryBasePtr trajectory = RaveCreateTrajectory(GetEnv(),"");
		trajectory->Init(robot->GetActiveConfigurationSpecification());
        while(goalReachedFlag==0 || difftime(now,start)<180)
        {	
        	std::vector<dReal> randConfig=goalconfig;
            srand(time(NULL));
            if( ((float) rand()) / ((float) RAND_MAX) > goalBias )
            {	
		        randConfig = getRandomSample(lower, upper);
            }
	        // -0.564602 -- 2.13539
			// -0.3536 -- 1.2963
			// -2.12131 -- -0.15
			// -2.00001 -- -0.1
			// -10000 -- 10000
			// -2.00001 -- -0.1
			// -10000 -- 10000
	        int nearestNodeConfigIndex = NearestNeighborNode(tree._nodes, randConfig);
	        // int saveIndexToDelete = nearestNodeConfigIndex;
	        configurationType nearestNodeConfig = tree._nodes[nearestNodeConfigIndex]->getConfig();// nearestNodeConfig=nearestNodeConfig1;//(NearestNeighborNode(tree._nodes, randConfig))-> getConfig(), nearestNodeConfig=nearestNodeConfig1;
	        while (nearestNodeConfig!=randConfig){ // extend tree untill it connects to random sample
	            std::vector<configurationType> elementArray;
	        	RRTNode* oldNearestNode = nearestNode;
	        	for (unsigned iter =0; iter<nearestNodeConfig.size(); iter++){
	        		if (fabs(nearestNodeConfig[iter] - randConfig[iter] > step)){
	        			if(nearestNodeConfig[iter] > randConfig[iter]) nearestNodeConfig[iter]-=step;
	        			else nearestNodeConfig[iter]+=step;
	        		}else{ nearestNodeConfig[iter]=randConfig[iter]; }
	        	}
	        	


            	if (validConfig(nearestNodeConfig,upper, lower)){
	            		// {elementArray.push_back(nearestNodeConfig);}
	            	
			        robot -> SetActiveDOFValues(nearestNodeConfig);
		            if(env->CheckCollision(robot) || robot->CheckSelfCollision()){   
		        		std::cout<<"breaking "<<std::endl;
		        		// deleteElements(elementArray, tree._nodes);
		        		// deleteBranchFromIndex(tree._nodes[saveIndexToDelete], tree._nodes[nearestNodeConfigIndex]);
		                break;
		            }  else{

			        	nearestNode = new RRTNode(nearestNodeConfig); // update nearest node if no collision
		            	tree.addNode(nearestNode,oldNearestNode); 
		            	std::cout<<tree._nodes.size()<<std::endl;//adding vertex
		            	// nearestNodeConfigIndex = getIndex(nearestNode, tree._nodes);
			        	handles.push_back(env->plot3(&(getEEPosition(robot, nearestNode->getConfig()))[0], 1, 1,5,colour, 0, true));
		            }
		        }
		        // }
		        // else{

		        // 	break;
		        // }
		    	// std::cout<<tree._nodes.size()<<"sakjfkhafand"<<std::endl;
	        }
        	for (unsigned iter =0; iter<7; iter++)
	    		std::cout<<nearestNodeConfig[iter]<<" ";
    		std::cout<<std::endl;
    		distToGoal=getDistance(nearestNode->getConfig(), goalconfig);
    		std::cout<<"dist to goal "<<distToGoal<<std::endl;

    		if (distToGoal==0){
	        	std::cout<<"success!"<<std::endl;
	        	goalReachedFlag=1;
	        	break;
	        	// return true;	        
	        }
	       time(&now);
       	// std::cout<<"reached traj section!"<<std::endl;
		// std::vector<RRTNode*> pathToRoot = tree.getPathToRoot(nearestNode);
	}

	if (goalReachedFlag==1){


    	std::cout<<"reached traj section!"<<std::endl;
    	std::cout<<tree._nodes.size()<<" tree"<<std::endl;
		std::vector<RRTNode*> pathToRoot = tree.getPathToRoot(nearestNode);
    	std::cout<<pathToRoot.size()<< " path" <<std::endl;

		for (unsigned int iter =0; iter<pathToRoot.size();iter++){
			// std::cout<<" path "<< pathToRoot[iter]->getConfig()[0]<<std::endl;
			trajectory ->Insert(0,pathToRoot[iter]->getConfig());
		}
    	std::cout<<"out of traj section!"<<std::endl;

		planningutils::RetimeActiveDOFTrajectory(trajectory, robot, false, 0.10);
		robot->GetController()->SetPath(trajectory);
	}
	   	return true;
   // }
   }

    bool ShortCutSmoothing(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        
        sout << "output wheeeee";
        std::cout<<input<<std::endl;
        sinput >> input;
        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "mynewmodule" ) {
        return InterfaceBasePtr(new MyNewModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("MyNewModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
