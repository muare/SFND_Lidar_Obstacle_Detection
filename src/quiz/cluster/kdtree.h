/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		kd_insert_helper(&root, point, id, 0);
	}

	void kd_insert_helper(Node** n, std::vector<float> point, int id, int depth)
	{
		if(*n == NULL)
		{
			*n = new Node(point, id);
			return;
		}

		int cd = depth%2;

		if(point[cd] < (*n)->point[cd])
		{
			kd_insert_helper(&((*n)->left), point, id, depth+1);
		}
		else
		{
			kd_insert_helper(&((*n)->right), point, id, depth+1);
		}


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		kd_search_helper(root, target, distanceTol,0, ids);
		return ids;
	}

	void kd_search_helper(Node* n, std::vector<float> target, float distanceTol, int depth,std::vector<int>& ids)
	{
		if(n == NULL)
		{
			return;
		}

		if ( (n->point[0] <= target[0] + distanceTol && n->point[0] >= target[0] - distanceTol) && 
			 (n->point[1] <= target[1] + distanceTol && n->point[1] >= target[1] - distanceTol) &&
			 (sqrt((n->point[0]-target[0])*(n->point[0]-target[0]) + (n->point[1]-target[1])*(n->point[1]-target[1])) < distanceTol)
			)
		{
			ids.push_back(n->id);
		}

		int cd = depth % 2;
		
		if(n->point[cd] < target[cd] + distanceTol)
		{
			kd_search_helper(n->right, target, distanceTol, depth+1, ids);
		}
		
		if(n->point[cd] > target[cd] - distanceTol)
		{
			kd_search_helper(n->left, target, distanceTol, depth+1, ids);
		}

	}
	

};




