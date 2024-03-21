#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <queue>
#include <cmath>
#include <vector>

#include <unistd.h>

using namespace std;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using KdTree = pcl::search::KdTree<pcl::PointXYZ>;
using KdTreePtr = KdTree::Ptr;
using Graph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>;

class ellipsoidalClustering : public pcl::PCLBase<pcl::PointXYZ>
{
    using Base = pcl::PCLBase<pcl::PointXYZ>;
    using Base::deinitCompute;
    using Base::indices_;
    using Base::initCompute;
    using Base::input_;

public:
    ellipsoidalClustering(float rho, float theta, float phi)
        : rho_(rho), theta_(theta), phi_(phi)
    {
    }

    std::array<std::array<int, 2048>, 64> segment()
    {
        std::array<std::array<int, 2048>, 64> ID_matrix;
        ID_matrix.fill({});
        if (!initCompute() || input_->empty() || indices_->empty())
            return ID_matrix;

        if (!tree_)
            tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);

        tree_->setInputCloud(input_, indices_);

        std::vector<int> L_ID(input_->size(), -1);
        std::vector<bool> extended_flags(input_->size(), false);

        std::vector<int> nn_indices;
        std::vector<float> nn_distances;

        Graph G;
        std::queue<int> L_Q;

        int ID = 0;
        float item_b = tan(theta_ * M_PI / 180.0f / 2.0);
        float item_c = tan(phi_ * M_PI / 180.0f / 2.0);
        float a = rho_ / 2.0;

        for (auto i_p : *indices_)
        {
            // if (i_p == indices_->size() - 1)
            // {
            //     cout << "pass" << endl;
            // }
            if (extended_flags.at(i_p))
                continue;

            boost::add_edge(ID, ID, G);

            L_Q.push(i_p);
            while (!L_Q.empty())
            {
                auto i_q = L_Q.front();
                L_Q.pop();
                if (extended_flags.at(i_q))
                    continue;

                float m = input_->points[i_q].x;
                float n = input_->points[i_q].y;
                float v = input_->points[i_q].z;

                float d = sqrt(m * m + n * n);
                float b = item_b * d;
                float c = item_c * d;
                float lambda = atan2(n, m);

                // Find the longest and the shortest ellipsoidal axes
                float max_axis = a > b ? a : b;
                max_axis = max_axis > c ? max_axis : c;

                float min_axis = a < b ? a : b;
                min_axis = min_axis < c ? min_axis : c;

                // Find the points that MIGHT be in the ellipsoidal neighbor
                tree_->radiusSearch(i_q, max_axis, nn_indices, nn_distances);

                for (std::size_t i = 0; i < nn_indices.size(); ++i)
                {
                    auto i_n = nn_indices.at(i);
                    auto q_label = L_ID.at(i_n);

                    float x = input_->points[i_n].x;
                    float y = input_->points[i_n].y;
                    float z = input_->points[i_n].z;

                    double item1 = pow((((x - m) * cos(lambda) + (y - n) * sin(lambda)) / a), 2);
                    double item2 = pow((((m - x) * sin(lambda) + (y - n) * cos(lambda)) / b), 2);
                    double item3 = pow(((z - v) / c), 2);

                    // Find the points that ARE in the ellipsoidal neighbor
                    if (item1 + item2 + item3 > 1.0)
                        continue;

                    // Mark the IDs of neighboring instances
                    if (q_label != -1 && q_label != ID)
                        boost::add_edge(ID, q_label, G);

                    if (extended_flags.at(i_n))
                        continue;

                    L_ID.at(i_n) = ID;

                    // Stop finding the neighboring points of close points
                    if (nn_distances.at(i) <= min_axis)
                        extended_flags.at(i_n) = true;
                    else
                        L_Q.push(i_n);
                }

                extended_flags.at(i_q) = true; // 不在論文中 這表示當作主角的不用在被搜索 但是它沒有被賦予類別
            }                                  // 搜索過不再次搜索

            ID++;
        }

        // Remap instance ID by looking up the graph
        std::vector<int> ID_mapper(boost::num_vertices(G)); // num_vertices 計算圖中有多少頂點 這也代表有多少類別(cluster了幾類)
        // cout << ID_mapper.size() << endl;

        auto num_components = boost::connected_components(G, ID_mapper.data()); // ID_mapper 用來儲存每個頂點屬於哪個連通分量（相連算同個區塊）
        // cout << num_components << endl;                                         // 重疊的算一類，總共有幾類

        for (auto index : *indices_) // 所有的點索引
        {
            auto ID = L_ID.at(index);            // 該點的類別
            auto remapped_ID = ID_mapper.at(ID); // 該ID屬於哪個連通分量

            int column = int(index % 2048);
            int row = int(index / 2048);
            ID_matrix[row][column] = remapped_ID + 1;  // 2048個點一列 這應該是要配合輸入的點的格式
        }

        deinitCompute();
        return ID_matrix;
    }

    std::array<std::array<int, 2048>, 64> ellipsoidalClustering_main(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud)
    {
        PointCloudPtr cloud(new PointCloud);
        cloud->clear();
        cloud->width = inputCloud->size();
        cloud->height = 1;
        cloud->is_dense = false;

        for (int i = 0; i < inputCloud->size(); i++)
        {
            pcl::PointXYZ point;
            point.x = inputCloud->points[i].x;
            point.y = inputCloud->points[i].y;
            point.z = inputCloud->points[i].z;
            cloud->points.push_back(point);
        }

        KdTreePtr tree(new KdTree);

        input_ = cloud;
        tree_ = tree;

        std::array<std::array<int, 2048>, 64> ID_matrix = segment();

        cloud->clear();

        return ID_matrix;
    }

private:
    float rho_ = 2.0;
    float theta_ = 2.0;
    float phi_ = 7.5;
    KdTreePtr tree_;
};