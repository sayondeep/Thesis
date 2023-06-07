#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

// Helper function to check if all elements in a matrix are equal
template<typename T>
bool checkEqual(const std::vector<std::vector<T>>& matrix)
{
    if (matrix.empty() || matrix[0].empty())
        return true;

    const T& first = matrix[0][0];

    for (const auto& row : matrix)
    {
        if (std::any_of(row.begin(), row.end(), [&first](const T& value) { return value != first; }))
            return false;
    }

    return true;
}

// Helper function to split an image into 4 quadrants
template<typename T>
std::vector<std::vector<T>> split4(const std::vector<std::vector<T>>& img)
{
    std::vector<std::vector<T>> split_img(4, std::vector<T>());

    int rows = img.size();
    int cols = img[0].size();

    int half_rows = rows / 2;
    int half_cols = cols / 2;

    // Split image into quadrants
    for (int i = 0; i < half_rows; ++i)
    {
        split_img[0].insert(split_img[0].end(), img[i].begin(), img[i].begin() + half_cols);
        split_img[1].insert(split_img[1].end(), img[i].begin() + half_cols, img[i].end());
    }

    for (int i = half_rows; i < rows; ++i)
    {
        split_img[2].insert(split_img[2].end(), img[i].begin(), img[i].begin() + half_cols);
        split_img[3].insert(split_img[3].end(), img[i].begin() + half_cols, img[i].end());
    }

    return split_img;
}

// Helper function to concatenate 4 quadrants into a single image
template<typename T>
std::vector<std::vector<T>> concatenate4(const std::vector<std::vector<T>>& nw,
                                         const std::vector<std::vector<T>>& ne,
                                         const std::vector<std::vector<T>>& sw,
                                         const std::vector<std::vector<T>>& se)
{
    std::vector<std::vector<T>> result;

    result.insert(result.end(), nw.begin(), nw.end());
    result.insert(result.end(), ne.begin(), ne.end());

    result.insert(result.end(), sw.begin(), sw.end());
    result.insert(result.end(), se.begin(), se.end());

    return result;
}

class QuadTree
{
private:
    static int tile_num;
    static std::unordered_map<int, std::unordered_map<int, QuadTree>> tiles;
    static std::unordered_map<int, int> count;

public:
    int level;
    std::vector<std::vector<int>> img;
    bool final;
    QuadTree* north_west;
    QuadTree* north_east;
    QuadTree* south_west;
    QuadTree* south_east;

    QuadTree() : level(0), final(true),
                 north_west(nullptr), north_east(nullptr),
                 south_west(nullptr), south_east(nullptr)
    {
    }

    QuadTree insert(const std::vector<std::vector<int>>& img, const std::vector<std::vector<int>>& matrix, int level = 0)
    {
        this->level = level;
        this->img = img;
        QuadTree::tiles[level][QuadTree::count[level]] = *this;
        QuadTree::count[level]++;
        this->final = true;

        if (!checkEqual(matrix))
        {
            std::vector<std::vector<int>> split_img = split4(img);
            this->final = false;

            std::vector<std::vector<int>> n_w, n_e, s_w, s_e;
            std::tie(n_w, n_e, s_w, s_e) = split(matrix, matrix.size() / 2, matrix[0].size() / 2);

            this->north_west = new QuadTree();
            *this->north_west = this->north_west->insert(split_img[0], n_w, level + 1);

            this->north_east = new QuadTree();
            *this->north_east = this->north_east->insert(split_img[1], n_e, level + 1);

            this->south_west = new QuadTree();
            *this->south_west = this->south_west->insert(split_img[2], s_w, level + 1);

            this->south_east = new QuadTree();
            *this->south_east = this->south_east->insert(split_img[3], s_e, level + 1);
        }

        return *this;
    }

    std::vector<std::vector<int>> get_image(int level)
    {
        if (this->final || this->level == level)
            return border(this->img);

        return concatenate4(
            this->north_west->get_image(level),
            this->north_east->get_image(level),
            this->south_west->get_image(level),
            this->south_east->get_image(level));
    }

private:
    // Helper function to split a matrix into four quadrants
    template<typename T>
    std::tuple<std::vector<std::vector<T>>, std::vector<std::vector<T>>,
               std::vector<std::vector<T>>, std::vector<std::vector<T>>>
    split(const std::vector<std::vector<T>>& matrix, int row_split, int col_split)
    {
        std::vector<std::vector<T>> nw(matrix.begin(), matrix.begin() + row_split);
        std::vector<std::vector<T>> ne(matrix.begin(), matrix.begin() + row_split);
        std::vector<std::vector<T>> sw(matrix.begin() + row_split, matrix.end());
        std::vector<std::vector<T>> se(matrix.begin() + row_split, matrix.end());

        for (auto& row : nw)
            row.resize(col_split);

        for (auto& row : ne)
            row.erase(row.begin(), row.begin() + col_split);

        for (auto& row : sw)
            row.resize(col_split);

        for (auto& row : se)
            row.erase(row.begin(), row.begin() + col_split);

        return std::make_tuple(nw, ne, sw, se);
    }

    // Helper function to add a border around an image
    std::vector<std::vector<int>> border(const std::vector<std::vector<int>>& img)
    {
        int rows = img.size();
        int cols = img[0].size();

        std::vector<std::vector<int>> bordered_img(rows + 2, std::vector<int>(cols + 2));

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                bordered_img[i + 1][j + 1] = img[i][j];
            }
        }

        return bordered_img;
    }
};

int QuadTree::tile_num = 0;
std::unordered_map<int, std::unordered_map<int, QuadTree>> QuadTree::tiles;
std::unordered_map<int, int> QuadTree::count;

int main()
{
    // Test the QuadTree class
    QuadTree quadtree;

    std::vector<std::vector<int>> img = { { 1, 1, 1, 1 },
                                          { 1, 0, 0, 1 },
                                          { 1, 0, 0, 1 },
                                          { 1, 1, 1, 1 } };

    std::vector<std::vector<int>> matrix = { { 1, 1 },
                                             { 1, 1 } };

    quadtree.insert(img, matrix);

    int level = 1;
    std::vector<std::vector<int>> result = quadtree.get_image(level);

    // Print the result
    for (const auto& row : result)
    {
        for (const auto& value : row)
        {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
