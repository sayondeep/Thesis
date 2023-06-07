// Define the structure for a point
struct Point {
    double x;
    double y;
};

// Define the structure for a quadtree node
struct QuadTreeNode {
    Point point;
    QuadTreeNode* topLeft;
    QuadTreeNode* topRight;
    QuadTreeNode* bottomLeft;
    QuadTreeNode* bottomRight;

    QuadTreeNode(Point p) : point(p), topLeft(nullptr), topRight(nullptr), bottomLeft(nullptr), bottomRight(nullptr) {}
};

// Define the quadtree class
class QuadTree {
public:
    QuadTree(double xMin, double xMax, double yMin, double yMax) : root(nullptr), xMin(xMin), xMax(xMax), yMin(yMin), yMax(yMax) {}

    void insert(Point point) {
        root = insertHelper(root, point, xMin, xMax, yMin, yMax);
    }

    QuadTreeNode* search(Point point) {
        return searchHelper(root, point);
    }

private:
    QuadTreeNode* root;
    double xMin;
    double xMax;
    double yMin;
    double yMax;

    QuadTreeNode* insertHelper(QuadTreeNode* node, Point point, double xMin, double xMax, double yMin, double yMax) {
        if (node == nullptr) {
            return new QuadTreeNode(point);
        }

        if (point.x < (xMin + xMax) / 2) {
            if (point.y < (yMin + yMax) / 2) {
                node->bottomLeft = insertHelper(node->bottomLeft, point, xMin, (xMin + xMax) / 2, yMin, (yMin + yMax) / 2);
            } else {
                node->topLeft = insertHelper(node->topLeft, point, xMin, (xMin + xMax) / 2, (yMin + yMax) / 2, yMax);
            }
        } else {
            if (point.y < (yMin + yMax) / 2) {
                node->bottomRight = insertHelper(node->bottomRight, point, (xMin + xMax) / 2, xMax, yMin, (yMin + yMax) / 2);
            } else {
                node->topRight = insertHelper(node->topRight, point, (xMin + xMax) / 2, xMax, (yMin + yMax) / 2, yMax);
            }
        }

        return node;
    }

    QuadTreeNode* searchHelper(QuadTreeNode* node, Point point) {
        if (node == nullptr || (node->point.x == point.x && node->point.y == point.y)) {
            return node;
        }

        if (point.x < (xMin + xMax) / 2) {
            if (point.y < (yMin + yMax) / 2) {
                return searchHelper(node->bottomLeft, point);
            } else {
                return searchHelper(node->topLeft, point);
            }
        } else {
            if (point.y < (yMin + yMax) / 2) {
                return searchHelper(node->bottomRight, point);
            } else {
                return searchHelper(node->topRight, point);
            }
        }
    }
};
