#include <bits/stdc++.h>
#include <simulation.hpp>
#include <types.hpp>

bool running = false;
bool setted = false;

template <std::size_t N, std::size_t K>
void setTypeFastFixed(auto &func, int n, int k)
{
    if (n == N && k == K && !setted)
    {
        setted = true;
        func.template operator()<FastFixed<N, K>>();
    }
}

template <std::size_t N, std::size_t K>
void setTypeFixed(auto &func, int n, int k)
{
    if (n == N && k == K && !setted)
    {
        setted = true;
        func.template operator()<Fixed<N, K>>();
    }
}

void setTypeFloat(auto &func, std::string type)
{
    if (type == "FLOAT" && !setted)
    {
        setted = true;
        func.template operator()<float>();
    }
}

void setTypeDouble(auto &func, std::string type)
{
    if (type == "DOUBLE" && !setted)
    {
        setted = true;
        func.template operator()<double>();
    }
}

void setType(std::string type, auto func)
{
    size_t n = 0, k = 0;
    bool f;
    if (type != "FLOAT" && type != "DOUBLE")
    {
        int res = sscanf(type.c_str(), "FIXED(%zu, %zu)", &n, &k);
        if (res != 2)
        {
            int res = sscanf(type.c_str(), "FAST_FIXED(%zu, %zu)", &n, &k);
            if (res != 2)
            {
                throw std::runtime_error("Undefined type\n");
                return;
            }
        }
    }
    setted = false;
#define FIXED(N, K) setTypeFixed<N, K>(func, n, k)
#define FAST_FIXED(N, K) setTypeFastFixed<N, K>(func, n, k)
#define FLOAT setTypeFloat(func, type)
#define DOUBLE setTypeDouble(func, type)
    TYPES;
#undef FIXED
#undef FAST_FIXED
#undef FLOAT
#undef DOUBLE
    if (!setted)
    {
        throw std::runtime_error("Missing type\n");
    }
}

template <size_t N, size_t M, typename T1, typename T2, typename T3>
void run_static(size_t cur_n, size_t cur_m, FileData file_data)
{
    if (cur_n == N && cur_m == M && !running)
    {
        running = true;
        SIM::StaticSimulation<T1, T2, T3, N, M>(file_data).run();
    }
}

int main(int argc, char *argv[])
{
#ifndef TYPES
    throw std::runtime_error("DTYPES not defined");
#endif // DTYPES

#ifndef SIZES
    throw std::runtime_error("DSIZES not defined");
#endif // SIZES

    std::string ptype = "", vtype = "", vflowtype = "", filename = "";

    for (int i = 1; i < argc; ++i)
    {
        std::string s = argv[i];
        int ind = s.find('=');

        if (ind <= 0)
        {
            continue;
        }

        std::string s1 = s.substr(0, ind);
        std::string s2 = s.substr(ind + 1, s.size() - ind - 1);

        // std::cout << s1 << ": " << s2 << '\n';

        if (s1 == "--p-type")
            ptype = s2;
        else if (s1 == "--v-type")
            vtype = s2;
        else if (s1 == "--v-flow-type")
            vflowtype = s2;
        else if (s1 == "--file-name")
            filename = s2;
    }
    // std::cout << ptype << '\n' << vtype << '\n' << vflowtype << '\n' << filename << '\n';

    if (ptype.empty() || vtype.empty() || vflowtype.empty() || filename.empty())
    {
        throw std::runtime_error("Please provide all required parameters: --p-type, --v-type, --v-flow-type, --file-name\n");
        exit(-1);
    }
    FileData file_data(filename);

    size_t cur_n = file_data.N;
    size_t cur_m = file_data.M;

    // std::cout << cur_n << ' ' << cur_m << '\n';
    // SIM::DynamicSimulation<Fixed<>, Fixed<>, Fixed<>>(file_data).run();
    setType(ptype, [&]<typename T1>()
            { setType(vtype, [&]<typename T2>()
                      { setType(vflowtype, [&]<typename T3>()
                                {
            // std::cout << "GOOD ATTEMPT!\n";

#define S(N, M) run_static<N, M, T1, T2, T3>(cur_n, cur_m, file_data)
                SIZES;
#undef S

                if(!running) SIM::DynamicSimulation<T1, T2, T3>(file_data).run(); }); }); });
}