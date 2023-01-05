#include <iostream>
#include <vector>
#include <cstring>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>




using namespace std;

typedef std::chrono::high_resolution_clock clock_;
typedef std::chrono::duration<double, std::ratio<1> > second_;
//typedef clock_::duration > second_;

class ParticleHash {
    private:
        const int HASH_CELL_SIZE = 10;
        const int PARTICLE_MAX_AMOUNT_IN_CELL = 10;

        int hash_table_size;
        vector<int> hash_table;
        vector<int> particle_map;

        vector<int> query_result;
        int query_amount;

    public:
        void CreateHash(int particle_amount, vector<float> particle_data_position_x, vector<float> particle_data_position_y) {
            
            hash_table_size = 640*480 / HASH_CELL_SIZE; //particle_amount * 2;

            hash_table.resize(hash_table_size + 1);//new int[hash_table_size + 1]{0};
            particle_map.clear();
            particle_map.resize(particle_amount, 0);//new int[particle_amount]{0};

            // Add particle counts to hash table
            for (int particle_id = 0; particle_id < particle_amount; particle_id++) {
                int cell = hashPos(particle_data_position_x[particle_id], particle_data_position_y[particle_id]);
                hash_table[cell] += 1;
                //cout << "Particle " << particle_id << " in " << particle_data_position_x[particle_id] << "," << particle_data_position_y[particle_id] << " bucketed to cell " << cell << endl;
            }

            // Calculate partial sum
            int sum = 0;
            for (int hash_cell = 0; hash_cell <= hash_table_size; hash_cell++) {
                sum += hash_table[hash_cell];
                hash_table[hash_cell] = sum;
            }

            // Fill in object ids to particle map
            for (int particle_id = 0; particle_id < particle_amount; particle_id++) {
                int cell = hashPos(particle_data_position_x[particle_id], particle_data_position_y[particle_id]);
                hash_table[cell]--;
                particle_map[hash_table[cell]] = particle_id;
                
            }
            /*for (int particle_id = 0; particle_id < particle_amount; particle_id++) {
                cout << particle_id << ": " << particle_map[particle_id] << endl;
            }*/
            hash_table.clear();
    }

    int Query(int x, int y, int  up=0, int down=1, int left=0, int right=1) {
            int x_left = cellCoord(x) - left;
            int x_right = cellCoord(x) + right;
            int y_up = cellCoord(y) - up;
            int y_down = cellCoord(y) + down;
            int cell_amount = (1 + up + down) * (1 + left + right);

            query_amount = 0;

            //cout << cell_amount * PARTICLE_MAX_AMOUNT_IN_CELL << endl;
            query_result.clear();
            query_result.resize(cell_amount * PARTICLE_MAX_AMOUNT_IN_CELL);

            for (int xi = x_left; xi <= x_right; xi++) {
                for (int yi = y_up; yi <= y_down; yi++) {
                    int cell = hashCoords(xi, yi);
                    int start = hash_table[cell];
                    int end = hash_table[cell + 1];

                    //Go through all indexes until next cell
                    for (int i = start; i < end; i++) {
                        query_result[query_amount] = particle_map[i];
                        query_amount++;
                    }
                }
            }
            return query_amount;
        }

    int GetQueryResult(int index) {
        return query_result[index];
    } 

    int GetQueryAmount() {
        return query_amount;
    } 

    int cellCoord(int coord) {
        return floor(coord / HASH_CELL_SIZE);
    }

    int hashCoords(int x, int y) {
        int h = (x * 92837111) ^ (y * 689287499);
        return abs(h) % hash_table_size;
    }

    int hashPos(float x, float y) {
        return hashCoords(cellCoord(x), cellCoord(y));
    }
};


class PhysicsSolver {
    public:
        static const int X = 0;
        static const int Y = 1;

        static const int PARTICLE_MAX_AMOUNT = 100000;
        static const int PARTICLE_MAX_RADIUS = 5;
        
        static const int MAX_TICKRATE = 60;
    private:
        int particle_amount = 0;
        int updates_per_second;
        second_ delta_time;

        second_ average_update_time = second_(0);
        second_ average_real_delta_time = second_(0);
        second_ average_sleep_time = second_(0);

        int saved_time_count = 0;

        int simulation_area_width;
        int simulation_area_height;

        int cells_horizontal;
        int cells_vertical;
        int cells_total;
        
        ParticleHash particleHash = ParticleHash();

        clock_::time_point last_tick_time;

        vector<float> particle_data_position_x;
        vector<float> particle_data_position_y;
        vector<float> particle_data_velocity_x;
        vector<float> particle_data_velocity_y;
        vector<float> particle_data_radius;
        vector<int> particle_data_type;
        vector<float> particle_data_temperature;
    
    private:
        void CalculateDeltaTime() {
            second_ update_time;
            second_ real_delta_time;
            second_ sleep_time;

            if (updates_per_second == -1) {
                delta_time = chrono::duration_cast<second_>(clock_::now() - last_tick_time);
            } else {
                update_time = chrono::duration_cast<second_>(clock_::now() - last_tick_time);

                clock_::time_point target_time = last_tick_time;
                target_time += chrono::duration_cast<clock_::duration>(delta_time);

                chrono::time_point sleep_start_time = clock_::now();
                //this_thread::sleep_until(target_time);
                while (clock_::now() < target_time);
                sleep_time = chrono::duration_cast<second_>(clock_::now() - sleep_start_time);

                real_delta_time = chrono::duration_cast<second_>(clock_::now() - last_tick_time);
            }

            average_update_time = (saved_time_count * average_update_time + update_time) / (saved_time_count + 1);
            average_real_delta_time = (saved_time_count * average_real_delta_time + real_delta_time) / (saved_time_count + 1);
            average_sleep_time = (saved_time_count * average_sleep_time + sleep_time) / (saved_time_count + 1);
            saved_time_count++;
            last_tick_time = clock_::now();
        }

        void SleepMs(int ms) {
            chrono::time_point sleep_start_time = clock_::now();
            chrono::time_point sleep_end_time = sleep_start_time + chrono::milliseconds(ms);
            while (clock_::now() < sleep_end_time);
            return;
        }

        void ParticleCollisionDetection() {

            particleHash.CreateHash(particle_amount, particle_data_position_x, particle_data_position_y);
            int collision_count = 0;

            for (int particle=0; particle < particle_amount; particle++) {
                int checklist_size = particleHash.Query(particle_data_position_x[particle], particle_data_position_y[particle]);

                //cout << "Particle " << particle << " in " << particle_data_position_x[particle] << "," << particle_data_position_y[particle] << " checks particles: ";
                for (int c=0; c < checklist_size; c++) {
                    int checked_particle = particleHash.GetQueryResult(c);
                    if (checked_particle == particle)
                        continue;
                    if (PointsWithinDistance(particle_data_position_x[particle], particle_data_position_y[particle], particle_data_position_x[checked_particle], particle_data_position_y[checked_particle], particle_data_radius[particle] + particle_data_radius[checked_particle])) {
                        //cout << "Collision between " << particle << " (" << particle_data_position_x[particle] << "," << particle_data_position_y[particle] << ") and ";
                        //cout << checked_particle << " (" << particle_data_position_x[checked_particle] << "," << particle_data_position_y[checked_particle] << ")" << endl;
                        collision_count++;
                    }
                    //cout << checked_particle << ",";
                }
                //cout << endl;
            }
            cout << collision_count << " collisions" << endl;
        }

        bool PointsWithinDistance(float x1, float y1, float x2, float y2, float d) {
            float xd = x1 - x2;
            float yd = y1 - y2;
            if ((xd * xd + yd * yd) > d * d)
                return true; 
            return false;
        }

    public:
        PhysicsSolver(int width, int height) {
            cout << "Initializing physics solver, size " << width << "," << height << endl;
            simulation_area_width = width;
            simulation_area_height = height;

            last_tick_time = clock_::now();
        }
    
        bool CreateParticle(float x, float y, int particle_type, int velocity_x=0, int velocity_y=0, float radius=1, float temperature=20) {
            
            if (particle_amount >= PARTICLE_MAX_AMOUNT)    
                return false;
            if (radius > PARTICLE_MAX_RADIUS)
                throw "Radius too large";

            particle_data_position_x.push_back(x);
            particle_data_position_y.push_back(y);
            particle_data_velocity_x.push_back(velocity_x);
            particle_data_velocity_y.push_back(velocity_y);
            particle_data_radius.push_back(radius);
            particle_data_type.push_back(particle_type);
            particle_data_temperature.push_back(temperature);

            particle_amount += 1;

            return true;
        }

        void SetUpdatesPerSecond(int ups=-1) {
            if (ups == -1)
                return;
            updates_per_second = ups;
            delta_time = second_(1.0) / ups;
        }

        void RunSimulationStep() {
            
            ParticleCollisionDetection();
            CalculateDeltaTime();


            //SleepMs(2);
        }

        void PrintAverageTimes() {
            int average_ups = 1 / average_real_delta_time.count();
            int average_potential_ups = 1 / average_update_time.count();

            cout.precision(20);
            cout << endl;
            cout << "Simulation delta time: " << fixed << delta_time.count() << endl;
            cout << "Average update time per tick: " << fixed << average_update_time.count() << endl;
            cout << "Average sleep time per tick: " << fixed << average_sleep_time.count() << endl;
            cout << "Average total tick time: " << fixed << average_real_delta_time.count() << endl;
            cout << "Average updates per second: " << fixed << average_ups << endl;
            cout << "Average potential updates per second: " << fixed << average_potential_ups << endl;
        }

        int GetParticleAmount() {
            return particle_amount;
        }

};



int main()
{
    try {
        cout << "Starting simulation." << endl;

        srand(time(0));

        int area_width = 640;
        int area_height = 480;

        PhysicsSolver physicsSolver = PhysicsSolver(area_width, area_height);

        physicsSolver.SetUpdatesPerSecond(60);
        /*
        physicsSolver.CreateParticle(24, 24, 0);
        physicsSolver.CreateParticle(32, 24, 0, 1);
        physicsSolver.CreateParticle(32, 32, 0, 0, -1, 2);
        */

        for (int p = 0; p < 10000; p++) {
            physicsSolver.CreateParticle(rand() % area_width, rand() % area_height, 0);
        }

        cout << "Created " << physicsSolver.GetParticleAmount() << " particles" << endl;

        char input[5];
        int count = 0;
        while (true) {
            physicsSolver.RunSimulationStep();

            count++;
            if (count < 20)
                continue;
            count = 0;

            //TEMP BREAK
            break;

            cout << "Input x to exit" << endl;
            cin.getline(input, 5);
            if (strcmp(input, "x") == 0) 
                break;
        }
        physicsSolver.PrintAverageTimes();
        cout << "Done!" << endl;
    } catch (exception &exc) {
        cerr << exc.what();
    }
}