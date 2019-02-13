#ifndef MYHEADER_H
#define MYHEADER_H

// #include <vector>
static const float step = .2;
static const float goalBias = .6;
static const float threshold = .3;


using namespace OpenRAVE;
std::vector<GraphHandlePtr> handles;
float colour[4] = {1,0,0,1};
typedef std::vector<dReal> configurationType;

// namespace testn
class RRTNode {
// std::vector<float> _configuration;
public:
	RRTNode(configurationType inputConfig){
		_configuration = inputConfig;
		parent = NULL;
	}

	~RRTNode(){};

	RRTNode* getParentNode(){
		return parent;
	}
	void setParentNode(RRTNode* pNew){
		parent = pNew;
	}
	configurationType getConfig(){
		return _configuration;
	}
	void setConfig(configurationType inputConfig){
		_configuration = inputConfig;
	}


	//
private:
	RRTNode* parent;
	configurationType _configuration;	
};

class NodeTree{
public:
	NodeTree();
	NodeTree(RRTNode* root){
		_nodes.push_back(root);
	}

	~NodeTree(){};

	// methods to add, delete, get nodes, get path from root
	bool addNode(RRTNode* node, RRTNode* p){
		node->setParentNode(p);
		_nodes.push_back(node);

	}
	bool deleteNode(int index){
		_nodes.erase(_nodes.begin()+index);
	}
	RRTNode* getNode(int index){
		return _nodes[index];
	}
	// return path to root
	std::vector<RRTNode*> getPathToRoot(RRTNode* node){
		std::vector<RRTNode*> pathtoStartConfig;
		RRTNode* parent = node;
		while (parent!=NULL){
			pathtoStartConfig.push_back(parent);
			parent = parent->getParentNode();
		}
		return pathtoStartConfig;
	}
// private:
	std::vector<RRTNode*> _nodes;
	
};

int NearestNeighborNode(std::vector<RRTNode*> tree, const std::vector<float> nodeConfig);
float eucledian(const std::vector<float> node1, const std::vector<float> node2);
float getDistance(std::vector<dReal> node1, std::vector<dReal> node2);


#endif