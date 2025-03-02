//
// Created by jarry_goon on 24. 8. 6.
//

#ifndef LSH_HPP
#define LSH_HPP

#include <vector>
#include <array>
#include <unordered_map>

class LSH
{
    typedef std::vector<std::array<float, 2>>               hyperplaneTable;
    typedef std::unordered_map<size_t, std::vector<size_t>> hashTable;

    size_t num_hyperplanes;
    size_t num_tables;

    std::vector<hyperplaneTable> hyperplane_tables;
    std::vector<hashTable>       hash_tables;

    const float* x;
    const float* y;

public:
    LSH(const float* x, const float* y, size_t size, size_t num_hyperplanes, size_t num_tables);

    std::vector<size_t> nearest_neighbor(size_t idx, float eps);

private:
    std::vector<size_t> hash(float x, float y) const;
};

#endif //LSH_HPP
