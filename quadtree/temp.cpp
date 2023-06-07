#include <iostream>
#include <vector>
#include <numeric>
#include <functional>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <memory>

struct QuadTreeNode {
    int level;
    cv::Mat image;
    bool isFinal;
    std::shared_ptr<QuadTreeNode> northWest;
    std::shared_ptr<QuadTreeNode> northEast;
    std::shared_ptr<QuadTreeNode> southWest;
    std::shared_ptr<QuadTreeNode> southEast;

    QuadTreeNode(int level, const cv::Mat& image, bool isFinal)
        : level(level), image(image), isFinal(isFinal) {}
};

class QuadTree {
private:
    std::unordered_map<int, std::unordered_map<int, std::shared_ptr<QuadTreeNode>>> tiles;
    std::unordered_map<int, int> count;

    std::vector<cv::Mat> split4(const cv::Mat& image) {
        std::vector<cv::Mat> halfSplit;
        cv::split(image, halfSplit);

        std::vector<cv::Mat> result;
        for (const auto& half : halfSplit) {
            std::vector<cv::Mat> subSplit;
            cv::split(half, subSplit);
            for (const auto& sub : subSplit) {
                result.push_back(sub);
            }
        }
        return result;
    }

    cv::Mat concatenate4(const cv::Mat& northWest, const cv::Mat& northEast, const cv::Mat& southWest, const cv::Mat& southEast) {
        cv::Mat top, bottom;
        cv::hconcat(northWest, northEast, top);
        cv::hconcat(southWest, southEast, bottom);

        cv::Mat result;
        cv::vconcat(top, bottom, result);

        return result;
    }

    std::vector<cv::Mat> split(const cv::Mat& matrix, int nrows, int ncols) {
        std::vector<cv::Mat> result;
        for (int i = 0; i < matrix.rows; i += nrows) {
            for (int j = 0; j < matrix.cols; j += ncols) {
                result.push_back(matrix(cv::Range(i, i + nrows), cv::Range(j, j + ncols)));
            }
        }
        return result;
    }

    bool checkEqual(const std::vector<cv::Mat>& matrix) {
        cv::Mat first = matrix[0];
        for (const auto& m : matrix) {
            if (!std::equal(first.begin<uchar>(), first.end<uchar>(), m.begin<uchar>())) {
                return false;
            }
        }
        return true;
    }

public:
    std::shared_ptr<QuadTreeNode> insert(const cv::Mat& img, const std::vector<std::vector<int>>& matrix, int level = 0) {
        bool isFinal = checkEqual(split4(img));
        auto node = std::make_shared<QuadTreeNode>(level, img, isFinal);
        tiles[level][count[level]] = node;
        count[level]++;

        if (!isFinal) {
            auto splitImages = split4(img);
            auto subMatrices = split(matrix, matrix.size() / 2, matrix[0].size() / 2);

            node->northWest = insert(splitImages[0], subMatrices[0], level + 1);
            node->northEast = insert(splitImages[1], subMatrices[1], level + 1);
            node->southWest = insert(splitImages[2], subMatrices[2], level + 1);
            node->southEast = insert(splitImages[3], subMatrices[3], level + 1);
        }

        return node;
    }

    cv::Mat getImage(int level, const std::shared_ptr<QuadTreeNode>& node) {
        if (node->isFinal || node->level == level) {
            return node->image;
        }

        auto nwImg = getImage(level, node->northWest);
        auto neImg = getImage(level, node->northEast);
        auto swImg = getImage(level, node->southWest);
        auto seImg = getImage(level, node->southEast);

        return concatenate4(nwImg, neImg, swImg, seImg);
    }
};

// Usage example:
int main() {
    cv::Mat img = cv::imread("your_image_path.jpg", cv::IMREAD_COLOR);
    std::vector<std::vector<int>> matrix = {
        {0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0},
        {1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0},
        {1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1},
        {1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0},
        {1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0},
        {1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0},
        {1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0},
        {0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1},
        {1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0},
        {1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1},
        {0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1},
        {0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1},
        {1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0},
        {1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0}
    };

    QuadTree quadTree;
    auto rootNode = quadTree.insert(img, matrix);

    int levelToRetrieve = 3;
    cv::Mat result = quadTree.getImage(levelToRetrieve, rootNode);

    cv::imshow("Result", result);
    cv::waitKey(0);

    return 0;
}
