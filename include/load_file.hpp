#pragma once
#include <bits/stdc++.h>
#include <types.hpp>

struct FileData
{
    size_t N, M;
    std::array<Fixed<64, 32>, 256> rho;
    Fixed<64, 32> g;
    std::vector <std::vector <char>> field;

    FileData(){};

    FileData(const std::string& filename){
        load_file(filename);
    }
    void load_file(const std::string &filename)
    {
        freopen(filename.c_str(), "r", stdin);
        int count;
        std::cin >> N >> M >> g >> count;
        getchar();

        for(int i=0; i<count; ++i){
            char c = getchar();
            std::cin >> rho[c];
            getchar();
        }

        field.resize(N, std::vector<char>(M+1));

        for(int i=0; i<N; ++i){
            for(int j=0; j<M; ++j){
                char c = getchar();
                field[i][j] = c;
            }
            getchar();
            field[i][M]='\0';
        }

        fclose(stdin);
    }
};
