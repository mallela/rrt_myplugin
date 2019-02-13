#ifndef MYHEADERDEFS_C
#define MYHEADERDEFS_C

#include "MyHeader.h"
#include <math.h>
configurationType getRandomSample(std::vector<dReal> lo,std::vector<dReal> hi){
	configurationType randConfig;
    for (unsigned int i=0; i<7; i++){
    	srand(time(NULL));// seed the rand() to get different random nos every time
    	dReal rNum = lo[i] + static_cast <dReal> (rand()) /( static_cast <dReal> (RAND_MAX/(hi[i] - lo[i] )));
    	randConfig.push_back(rNum);
	}
	return randConfig;
}
bool validConfig(configurationType input, std::vector<dReal> lower, std::vector<dReal> upper){
    for (unsigned int iter=0;iter<input.size();iter++)
        if (input[iter]<=upper[iter] && input[iter]>=lower[iter])
        return true;
    return false;	
}
std::vector<float> getEEPosition(RobotBasePtr robot, configurationType config) {  

    RobotBase::RobotStateSaver save(robot);
    robot->SetActiveDOFValues(config);
    robot->SetActiveManipulator("leftarm");
    RobotBase::ManipulatorPtr maniPtr;
    maniPtr = robot->GetActiveManipulator();
    RaveVector<float> point = maniPtr->GetEndEffectorTransform().trans;
    std::vector<float> ee;
    ee.push_back(point.x);
    ee.push_back(point.y);
    ee.push_back(point.z);

    return ee;
}
void deleteBranchFromIndex(RRTNode* startN, RRTNode* endN){
    if (startN == endN ) return ;
    deleteBranchFromIndex(startN->getParentNode(),endN);
    delete startN;

}

void deleteElements(std::vector<configurationType> arrray, std::vector<RRTNode*> nodes){
	for (unsigned int iter = 0; iter<arrray.size(); iter++)
		for (unsigned int iter2 = 0; iter2<nodes.size(); iter2++){
			if (nodes[iter2]->getConfig() == arrray[iter]) {nodes[iter2]->setParentNode(NULL);}

	}
}
	int getIndex(RRTNode* node,std::vector<RRTNode*> nodes ){
		// std::vector<RRTNode*> it;
		int index = std::find(nodes.begin(), nodes.end(), node) - nodes.begin();
		std::cout<<"index "<<index<< " "<< nodes.size()<<std::endl;
		return index;
	}

float eucledian(const std::vector<dReal> node1, const std::vector<dReal> node2){
	float e=0;
	for (int iter = 0; iter <7; iter++)
		e+=pow(node1[iter]-node2[iter], 2);
	return sqrt(e);
}

float getDistance(std::vector<dReal> node1, std::vector<dReal> node2){
	float distance = 0;
	// float weights[7] = {0.491, 0.088, 0.065, 0.041, 0.032, 0.051, 0.016};
	float weights[7] = { 3.17104,2.75674, 2.2325,1.78948, 1.42903, 0.809013, 0.593084};

	for(unsigned int iter = 0; iter < node1.size(); iter++){
		distance = distance + weights[iter] * pow(node1[iter] - node2[iter],2);
	}
	return sqrt(distance);
};

int NearestNeighborNode(std::vector<RRTNode*> tree, const std::vector<dReal> nodeConfig){

	// std::cout<<"nearest neighbor begin "<<std::endl;
	// std::vector<dReal> temp ;//= [0,0,0,0,0,0,0];
	// for (int iter = 0; tree.size(); iter++)
		// temp.push_back(0.0);
	// std::cout<<eucledian(temp, nc)<<std::endl;

	float minDist = -1;
	unsigned int nodeIndex;
	float dist;
	for (unsigned int i=0; i<tree.size();i++){
		dist = eucledian(tree[i]->getConfig(),nodeConfig);
		if (minDist > dist || minDist == -1){
			minDist = dist;
			nodeIndex = i;
		}
		// std::cout << " nearest " << dist<<" "<< nodeIndex<<std::endl;
	}
	// return tree._nodes[nodeIndex];
	return nodeIndex;
}

#endif
