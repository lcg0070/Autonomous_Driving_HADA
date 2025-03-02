//
// Created by jarry_goon on 24. 8. 6.
//

#include "LSH.hpp"

#include <random>

LSH::LSH(const float* x, const float* y, size_t size, size_t num_hyperplanes, size_t num_tables)
    : num_hyperplanes(num_hyperplanes),
      num_tables(num_tables),
      hash_tables(std::vector<hashTable>(num_tables)),
      x(x),
      y(y)
{
    // 1. Create random number generator
    std::random_device              random_device;
    std::mt19937                    gen(random_device());
    std::normal_distribution<float> distribution(0.f, 1.f);

    // 2. Configuration hyperplane tables
    for(int table_idx = 0; table_idx < num_tables; table_idx++)
    {
        hyperplaneTable table;

        for(int plane_idx = 0; plane_idx < num_hyperplanes; plane_idx++)
            table.push_back({distribution(gen), distribution(gen)});

        hyperplane_tables.push_back(table);
    }

    // 3. Create hash table
    std::vector<size_t> hash_val;

    for(int idx = 0; idx < size; idx++)
    {
        hash_val = hash(x[idx], y[idx]);

        for(int table_idx = 0; table_idx < num_tables; table_idx++)
            hash_tables[table_idx][hash_val[table_idx]].push_back(idx);
    }
}

std::vector<size_t> LSH::nearest_neighbor(size_t idx, float eps)
{
    float               x_data    = x[idx];
    float               y_data    = y[idx];
    std::vector<size_t> hash_vals = hash(x_data, y_data);
    size_t              hash_val;

    std::vector<size_t> candidate_idx;

    for(int table_idx = 0; table_idx < num_tables; table_idx++)
    {
        hash_val = hash_vals[table_idx];

        if(hash_tables[table_idx].find(hash_val) == hash_tables[table_idx].end()) continue;

        for(int point_idx: hash_tables[table_idx][hash_val])
        {
            if(hypotf(x_data - x[point_idx], y_data - y[point_idx]) <= eps && idx != point_idx)
                candidate_idx.push_back(point_idx);
        }
    }

    return candidate_idx;
}

std::vector<size_t> LSH::hash(float x, float y) const
{
    std::vector<size_t> hash_vals(num_tables);
    size_t              hash_val = 0;

    int   hash_val_idx = 0;
    float dot_product;

    for(hyperplaneTable table: hyperplane_tables)
    {
        for(std::array<float, 2> hyperplane: table)
        {
            dot_product = x * hyperplane[0] + y * hyperplane[1];

            if(dot_product >= 0)
                hash_val = (hash_val << 1) | 1;
            else
                hash_val = hash_val << 1;
        }

        hash_vals[hash_val_idx++] = hash_val;
    }

    return hash_vals;
}
