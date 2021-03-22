/*  Michael Walden
    Lidar Project 1
    Sensor Fusion Nano Degree
    Udacity */

// Kd Tree 3D implementation

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	// std::vector<float> point;
	PointT point;
	int id;
	Node* left;
	Node* right;

	// Node(std::vector<float> arr, int setId)
	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	// Node* root;
	// Node<PointT>* root;
	typename Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	// void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	// template<typename PointT>
	void insertHelper(Node** node, uint depth, PointT point, int id)
	{
		if (*node == NULL)
			*node = new Node<PointT>(point,id);
		else
		{
			// Calculate current dim
			uint cd = depth % 3;

			// TESTING MARK
			if (cd == 0) // Dimension = x
			{
				if (point.x < ((*node)->point.x))
					insertHelper(&((*node)->left), depth+1, point, id);
				else
				{
					insertHelper(&((*node)->right), depth+1, point, id);				
				}
			}
			else if (cd == 1) // Dimension = y
			{
				if (point.y < ((*node)->point.y))
					insertHelper(&((*node)->left), depth+1, point, id);
				else
				{
					insertHelper(&((*node)->right), depth+1, point, id);				
				}
			}
			else // Dimension = z
			{
				if (point.z < ((*node)->point.z))
					insertHelper(&((*node)->left), depth+1, point, id);
				else
				{
					insertHelper(&((*node)->right), depth+1, point, id);				
				}
			}
			// END OF TEST

			// if (point[cd] < ((*node)->point[cd]))
			// 	insertHelper(&((*node)->left), depth+1, point, id);
			// else
			// {
			// 	insertHelper(&((*node)->right), depth+1, point, id);				
			// }
		}
	}

	// void insert(std::vector<float> point, int id)
	// template<typename PointT>
	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id);
	}

	// void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	// template<typename PointT>
	void searchHelper(PointT target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			// if ( 	((node->point[0]>=(target[0]-distanceTol)) && (node->point[0]<=(target[0]+distanceTol))) && 
			// 		((node->point[1]>=(target[1]-distanceTol)) && (node->point[1]<=(target[1]+distanceTol))) && 
			// 		((node->point[2]>=(target[2]-distanceTol)) && (node->point[2]<=(target[2]+distanceTol)))	) 
			// {
			// 	float distance = sqrt( 	(node->point[0]-target[0])*(node->point[0]-target[0]) + 
			// 							(node->point[1]-target[1])*(node->point[1]-target[1]) +
			// 							(node->point[2]-target[2])*(node->point[2]-target[2])	);
			// 	if (distance <= distanceTol)
			// 		ids.push_back(node->id);
			// }
			if ( 	((node->point.x>=(target.x-distanceTol)) && (node->point.x<=(target.x+distanceTol))) && 
					((node->point.y>=(target.y-distanceTol)) && (node->point.y<=(target.y+distanceTol))) && 
					((node->point.z>=(target.z-distanceTol)) && (node->point.z<=(target.z+distanceTol)))	) 
			{
				// float distance = sqrt( 	(node->point[0]-target[0])*(node->point[0]-target[0]) + 
				// 						(node->point[1]-target[1])*(node->point[1]-target[1]) +
				// 						(node->point[2]-target[2])*(node->point[2]-target[2])	);
				float distance = sqrt( 	(node->point.x-target.x)*(node->point.x-target.x) + 
										(node->point.y-target.y)*(node->point.y-target.y) +
										(node->point.z-target.z)*(node->point.z-target.z)	);
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			// // check across boundary
			// if ((target[depth%3]-distanceTol) <= node->point[depth%3])
			// 	searchHelper(target, node->left, depth+1, distanceTol, ids);
			// if ((target[depth%3]+distanceTol) > node->point[depth%3])
			// 	searchHelper(target, node->right, depth+1, distanceTol, ids);			

			// check across boundary
			if ( (depth%3) == 0 )	// Dimension = x
			{
				if ((target.x-distanceTol) <= node->point.x)
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				else
					searchHelper(target, node->right, depth+1, distanceTol, ids);			
			}
			else if ( (depth%3) == 1 )	// Dimension = y
			{
				if ((target.y-distanceTol) <= node->point.y)
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				else
					searchHelper(target, node->right, depth+1, distanceTol, ids);			
			}
			else	// Dimension = z
			{
				if ((target.z-distanceTol) <= node->point.z)
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				else
					searchHelper(target, node->right, depth+1, distanceTol, ids);			
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	// std::vector<int> search(std::vector<float> target, float distanceTol)
	// template<typename PointT>
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




