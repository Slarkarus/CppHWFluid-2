#pragma once
#include <bits/stdc++.h>
#include <types.hpp>
#include <load_file.hpp>

namespace SIM
{
    constexpr std::array<std::pair<int, int>, 4> deltas{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};

    template <typename T>
    using DynamicMatrix = std::vector<std::vector<T>>;

    template <typename T, size_t N, size_t M>
    using StaticMatrix = std::array<std::array<T, M>, N>;

    template <typename T, bool B, size_t N, size_t M>
    using CondStaticMatrix = std::conditional<B, StaticMatrix<T, N, M>, DynamicMatrix<T>>::type;

    template <typename PType, typename VType, typename VFlowType, size_t N, size_t M, bool is_static>
    class Simulation
    {
    private:
        size_t N_, M_;
        std::array<VType, 256> rho;

        CondStaticMatrix<PType, is_static, N, M> p, old_p;
        CondStaticMatrix<char, is_static, N, M> field;

        VType g;
        void init_field()
        {
            if constexpr (!is_static)
            {
                field.resize(N_, std::vector<char>(M_ + 1));
            }
        }
        void init_point()
        {
            if constexpr (!is_static)
            {
                p.resize(N_, std::vector<PType>(M_));
                old_p.resize(N_, std::vector<PType>(M_));
            }
            else
            {
                for (int i = 0; i < N; i++)
                {
                    for (int j = 0; j < M; j++)
                    {
                        p[i][j] = PType{};
                    }
                }
            }
        }

        template <typename T>
        struct VectorField
        {

            void init(std::size_t N__, std::size_t M__)
            {
                if constexpr (!is_static)
                {
                    v.resize(N__, std::vector<std::array<T, deltas.size()>>(M__));
                }
            }

            CondStaticMatrix<std::array<T, deltas.size()>, is_static, N, M> v;

            T &add(int x, int y, int dx, int dy, T dv)
            {
                return get(x, y, dx, dy) += dv;
            }

            T &get(int x, int y, int dx, int dy)
            {
                // size_t i = std::ranges::find(deltas, pair(dx, dy)) - deltas.begin();
                // assert(i < deltas.size());
                return v[x][y][((dy & 1) << 1) | (((dx & 1) & ((dx & 2) >> 1)) | ((dy & 1) & ((dy & 2) >> 1)))];
            }
        };

        VectorField<VType> velocity;
        VectorField<VFlowType> velocity_flow;

        CondStaticMatrix<int, is_static, N, M> last_use;
        // int last_use[N][M]{};
        void init_last_use()
        {
            if constexpr (!is_static)
            {
                last_use.resize(N_, std::vector<int>(M_));
            }
            else
            {
                for (int i = 0; i < N; ++i)
                {
                    for (int j = 0; j < M; ++j)
                    {
                        last_use[i][j] = 0;
                    }
                }
            }
        }
        int UT = 0;

        std::mt19937 rnd{1337};

        std::tuple<VFlowType, bool, std::pair<int, int>> propagate_flow(int x, int y, VFlowType lim)
        {
            last_use[x][y] = UT - 1;
            VFlowType ret = 0;
            for (auto [dx, dy] : deltas)
            {
                int nx = x + dx, ny = y + dy;
                if (field[nx][ny] != '#' && last_use[nx][ny] < UT)
                {
                    VType cap = velocity.get(x, y, dx, dy);
                    VFlowType flow = velocity_flow.get(x, y, dx, dy);
                    if (flow == cap)
                    {
                        continue;
                    }
                    // assert(v >= velocity_flow.get(x, y, dx, dy));
                    VFlowType vp = std::min(VType(lim), cap - VType(flow));
                    if (last_use[nx][ny] == UT - 1)
                    {
                        velocity_flow.add(x, y, dx, dy, vp);
                        last_use[x][y] = UT;
                        // cerr << x << " " << y << " -> " << nx << " " << ny << " " << vp << " / " << lim << "\n";
                        return {vp, 1, {nx, ny}};
                    }
                    auto [t, prop, end] = propagate_flow(nx, ny, vp);
                    ret += t;
                    if (prop)
                    {
                        velocity_flow.add(x, y, dx, dy, t);
                        last_use[x][y] = UT;
                        // cerr << x << " " << y << " -> " << nx << " " << ny << " " << t << " / " << lim << "\n";
                        return {t, prop && end != std::pair(x, y), end};
                    }
                }
            }
            last_use[x][y] = UT;
            return {ret, 0, {0, 0}};
        }

        static constexpr double max_double = (double)std::numeric_limits<double>::max();
        double random01()
        {
            return rnd() / max_double;
        }

        void propagate_stop(int x, int y, bool force = false)
        {
            if (!force)
            {
                bool stop = true;
                for (auto [dx, dy] : deltas)
                {
                    int nx = x + dx, ny = y + dy;
                    if (field[nx][ny] != '#' && last_use[nx][ny] < UT - 1 && velocity.get(x, y, dx, dy) > 0)
                    {
                        stop = false;
                        break;
                    }
                }
                if (!stop)
                {
                    return;
                }
            }
            last_use[x][y] = UT;
            for (auto [dx, dy] : deltas)
            {
                int nx = x + dx, ny = y + dy;
                if (field[nx][ny] == '#' || last_use[nx][ny] == UT || velocity.get(x, y, dx, dy) > 0)
                {
                    continue;
                }
                propagate_stop(nx, ny);
            }
        }

        VType move_prob(int x, int y)
        {
            VType sum = 0;
            for (size_t i = 0; i < deltas.size(); ++i)
            {
                auto [dx, dy] = deltas[i];
                int nx = x + dx, ny = y + dy;
                if (field[nx][ny] == '#' || last_use[nx][ny] == UT)
                {
                    continue;
                }
                auto v = velocity.get(x, y, dx, dy);
                if (v < 0)
                {
                    continue;
                }
                sum += v;
            }
            return sum;
        }

        struct ParticleParams
        {
            char type;
            PType cur_p;
            std::array<VType, deltas.size()> v;

            void swap_with(int x, int y, auto &p_, auto &velocity_, auto &field_)
            {
                std::swap(field_[x][y], type);
                std::swap(p_[x][y], cur_p);
                std::swap(velocity_.v[x][y], v);
            }
        };

        bool propagate_move(int x, int y, bool is_first)
        {
            last_use[x][y] = UT - is_first;
            bool ret = false;
            int nx = -1, ny = -1;
            do
            {
                std::array<VType, deltas.size()> tres;
                VType sum = 0;
                for (size_t i = 0; i < deltas.size(); ++i)
                {
                    auto [dx, dy] = deltas[i];
                    int nx = x + dx, ny = y + dy;
                    if (field[nx][ny] == '#' || last_use[nx][ny] == UT)
                    {
                        tres[i] = sum;
                        continue;
                    }
                    VType v = velocity.get(x, y, dx, dy);
                    if (v < 0)
                    {
                        tres[i] = sum;
                        continue;
                    }
                    sum += v;
                    tres[i] = sum;
                }

                if (sum == 0) ///
                {
                    break;
                }

                double p = ((double)sum) * random01();
                size_t d = std::ranges::upper_bound(tres, p) - tres.begin();

                auto [dx, dy] = deltas[d];
                nx = x + dx;
                ny = y + dy;
                assert(velocity.get(x, y, dx, dy) > 0 && field[nx][ny] != '#' && last_use[nx][ny] < UT);

                ret = (last_use[nx][ny] == UT - 1 || propagate_move(nx, ny, false));
            } while (!ret);
            last_use[x][y] = UT;
            for (size_t i = 0; i < deltas.size(); ++i)
            {
                auto [dx, dy] = deltas[i];
                int nx = x + dx, ny = y + dy;
                if (field[nx][ny] != '#' && last_use[nx][ny] < UT - 1 && velocity.get(x, y, dx, dy) < 0)
                {
                    propagate_stop(nx, ny);
                }
            }
            if (ret)
            {
                if (!is_first)
                {
                    ParticleParams pp{};
                    pp.swap_with(x, y, p, velocity, field);
                    pp.swap_with(nx, ny, p, velocity, field);
                    pp.swap_with(x, y, p, velocity, field);
                }
            }
            return ret;
        }

        // int dirs[N][M]{};
        CondStaticMatrix<double, is_static, N, M> dirs;

        void init_dirs()
        {
            if constexpr (!is_static)
            {
                dirs.resize(N_, std::vector<double>(M_));
            }
            else
            {
                for (int i = 0; i < N; i++)
                {
                    for (int j = 0; j < M; j++)
                    {
                        dirs[i][j] = 0;
                    }
                }
            }
        }
        void init_all(const FileData &data)
        {

            if constexpr (!is_static)
            {
                N_ = data.N;
                M_ = data.M;
            }

            std::cout << "Start init simulation\n";

            init_last_use();
            init_dirs();
            init_point();
            init_field();

            velocity.init(N_, M_);
            velocity_flow.init(N_, M_);

            for (int i = 0; i < 256; ++i)
            {
                rho[i] = VType(data.rho[i]);
            }
            g = VType(data.g);
            for (int i = 0; i < N_; ++i)
            {
                for (int j = 0; j < M_ + 1; ++j)
                {
                    field[i][j] = data.field[i][j];
                }
            }
            for (size_t x = 0; x < N_; ++x)
            {
                for (size_t y = 0; y < M_; ++y)
                {
                    if (field[x][y] == '#')
                        continue;
                    for (auto [dx, dy] : deltas)
                    {
                        dirs[x][y] += (field[x + dx][y + dy] != '#');
                    }
                }
            }
            std::cout << "End init simulation\n";
        }

    public:
        Simulation(FileData data) : N_(N), M_(M) { init_all(data); }

        void step(size_t TICK_NUM)
        {
            // std::cout << "Start simulation step\n";
            PType total_delta_p = 0;
            // Apply external forces
            for (size_t x = 0; x < N_; ++x)
            {
                for (size_t y = 0; y < M_; ++y)
                {
                    if (field[x][y] == '#')
                        continue;
                    if (field[x + 1][y] != '#')
                        velocity.add(x, y, 1, 0, g);
                }
            }

            // Apply forces from p
            // memcpy(old_p, p, sizeof(p));
            old_p = p;

            for (size_t x = 0; x < N_; ++x)
            {
                for (size_t y = 0; y < M_; ++y)
                {
                    if (field[x][y] == '#')
                        continue;
                    for (auto [dx, dy] : deltas)
                    {
                        int nx = x + dx, ny = y + dy;
                        if (field[nx][ny] != '#' && old_p[nx][ny] < old_p[x][y])
                        {
                            PType delta_p = old_p[x][y] - old_p[nx][ny];
                            PType force = delta_p;
                            VType &contr = velocity.get(nx, ny, -dx, -dy);
                            if (contr * VType(rho[(int)field[nx][ny]]) >= VType(force))
                            {
                                contr -= VType(force / PType(rho[(int)field[nx][ny]]));
                                continue;
                            }
                            force -= PType(contr * VType(rho[(int)field[nx][ny]]));
                            contr = 0;
                            velocity.add(x, y, dx, dy, VType(force / PType(rho[(int)field[x][y]])));
                            p[x][y] -= PType(force / dirs[x][y]);
                            total_delta_p -= PType(force / dirs[x][y]);
                        }
                    }
                }
            }

            // Make flow from velocities
            velocity_flow = {};
            velocity_flow.init(N_, M_);
            bool prop = false;
            do
            {
                UT += 2;
                prop = 0;
                for (size_t x = 0; x < N_; ++x)
                {
                    for (size_t y = 0; y < M_; ++y)
                    {
                        if (field[x][y] != '#' && last_use[x][y] != UT)
                        {

                            auto [t, local_prop, _] = propagate_flow(x, y, 1);
                            if (t > 0)
                            {
                                prop = 1;
                            }
                        }
                    }
                }
            } while (prop);

            // Recalculate p with kinetic energy
            for (size_t x = 0; x < N_; ++x)
            {
                for (size_t y = 0; y < M_; ++y)
                {
                    if (field[x][y] == '#')
                        continue;
                    for (auto [dx, dy] : deltas)
                    {
                        auto old_v = velocity.get(x, y, dx, dy);
                        auto new_v = VType(velocity_flow.get(x, y, dx, dy));
                        if (old_v > 0)
                        {
                            assert(new_v <= old_v);
                            velocity.get(x, y, dx, dy) = new_v;
                            PType force = (old_v - new_v) * VType(rho[(int)field[x][y]]);
                            if (field[x][y] == '.')
                                force *= 0.8;
                            if (field[x + dx][y + dy] == '#')
                            {
                                p[x][y] += force / dirs[x][y];
                                total_delta_p += force / dirs[x][y];
                            }
                            else
                            {
                                p[x + dx][y + dy] += force / dirs[x + dx][y + dy];
                                total_delta_p += force / dirs[x + dx][y + dy];
                            }
                        }
                    }
                }
            }

            UT += 2;
            prop = false;
            for (size_t x = 0; x < N_; ++x)
            {
                for (size_t y = 0; y < M_; ++y)
                {
                    if (field[x][y] != '#' && last_use[x][y] != UT)
                    {
                        if (random01() < move_prob(x, y))
                        {
                            prop = true;
                            propagate_move(x, y, true);
                        }
                        else
                        {
                            propagate_stop(x, y, true);
                        }
                    }
                }
            }
            if (prop)
            {
                std::cout << "Tick " << TICK_NUM << ":\n";
                for (size_t x = 0; x < N_; ++x)
                {
                    for (size_t y = 0; y < M_; ++y)
                    {
                        std::cout << field[x][y];
                    }
                    std::cout << "\n";
                }
            }
            // std::cout << "End simulation step\n";
        }

        void run()
        {
            std::cout << "Start run of simulation\n";
            for (size_t i = 0; i < 1000000; ++i)
            {
                step(i);
                if (i == 1000000 - 1)
                {
                    std::cout << "last step\n";
                }
            }
            std::cout << "End run of simulation\n";
        }
    };

template <typename PType, typename VType, typename VFlowType, size_t N, size_t M>
using StaticSimulation = Simulation<PType, VType, VFlowType, N, M, true>;

template <typename PType, typename VType, typename VFlowType>
using DynamicSimulation = Simulation<PType, VType, VFlowType, 0, 0, false>;
}; // namespace SIM