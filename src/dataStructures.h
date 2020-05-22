
#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <queue>

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};

struct BoundingBox { // bounding box around a classified object (contains both 2D and 3D data)
    
    int boxID; // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs
    
    cv::Rect roi; // 2D region-of-interest in image coordinates
    int classID; // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches; // keypoint matches enclosed by 2D roi
};

struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    std::vector<LidarPoint> lidarPoints;

    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
    std::map<int,int> bbMatches; // bounding box matches between previous and current frame
};

// Implement RingBuffer through inheritance from std::queue and slightly modyfing the push method.
template <typename T, std::size_t maxSize, typename Container = std::deque<T>>
class RingBuffer : public std::queue<T>
{
public:
    void push(const T& element)
    {
        if (Fifo::size() == maxSize) 
        {
            Fifo::c.pop_front();
        }
        Fifo::push(element);
    }

    typename Container::iterator begin() { return Fifo::c.begin(); }
    typename Container::iterator end() { return Fifo::c.end(); }
    typename Container::const_iterator begin() const { return Fifo::c.begin(); }
    typename Container::const_iterator end() const { return Fifo::c.end(); }

private:
    typedef std::queue<T> Fifo;
};

// Structure to represent node of kd tree
struct Node
{
	std::vector<double> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<double> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

    void insertHelper(Node *&node, uint depth, std::vector<double> point, int id) {

        // Empty node
        if(node == NULL)
            node = new Node(point,id);
        else {
            // Calculate current dimension
            uint dim = depth % 3;

            if(point[dim] < (node)->point[dim]) {
                insertHelper(node->left, depth+1, point, id);
            }
            else {
                insertHelper(node->right, depth+1, point, id);
            }
        }
    }

	void insert(std::vector<double> point, int id)
	{
		// DONE: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(root, 0, point, id);
	}

    void searchHelper(const std::vector<double> target, const Node* node, const int depth,
                        const double distanceTol, std::vector<int>& ids) const {
        if (node != NULL) {
          const double x_min = target[0] - distanceTol;
          const double x_max = target[0] + distanceTol;
          const double y_min = target[1] - distanceTol;
          const double y_max = target[1] + distanceTol;
          const double z_min = target[2] - distanceTol;
          const double z_max = target[2] + distanceTol;

          if (((node->point[0] >= x_min) && (node->point[0] <= x_max)) &&
              ((node->point[1] >= y_min) && (node->point[1] <= y_max)) &&
              ((node->point[2] >= z_min) && (node->point[2] <= z_max))) {
            double dist_x = node->point[0] - target[0];
            double dist_y = node->point[1] - target[1];
            double dist_z = node->point[2] - target[2];
            double distance =
                sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
            if (distance <= distanceTol) 
                ids.push_back(node->id);
          }

          if ((target[depth % 3] - distanceTol) < node->point[depth % 3])
            searchHelper(target, node->left, depth + 1, distanceTol, ids);
          if ((target[depth % 3] + distanceTol > node->point[depth % 3]))
            searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const std::vector<double> target, const double distanceTol) const
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);

        return ids;
    }
};

#endif /* dataStructures_h */
