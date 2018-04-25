#include "rtklib.h"
#include <cmath>
using namespace std;

#define SQR(x)     ((x)*(x))
#define PR_OUTLIER 20.0
#define PH_OUTLIER 0.4

static double gtime2sec(gtime_t time)
{
    return (double) time.time + time.sec;
}

static solbuf_t sol={0};
char *filenames = "/home/aleksey/dev/GitHub/rtklib-testing-suite/data/Corrupted_Real_Tests/Car/3/reference_solution.pos";
void read_pos_file(char *filename) {
    readsol(&filenames, 1, &sol);
    printf("SOOOL%d\n", sol.n);
    // for (int index = 0; index < sol.n; index++)
    // printf("stat %d\n", sol.data[index].stat);
}

bool get_sol_from_pos(extrpol_t &extr_data, gtime_t &time) {
    static int index = 0;
    for (int i = index; i < sol.n; i++) {
        if (gtime2sec(time) >= gtime2sec(sol.data[i].time)) {
            index = i; 
        }
    }
    //for (int index = 0; index < sol.n; index++)
    printf("STAT %d %d\n", sol.data[index].stat, index);
    // printf("POS %f %f %f", pos[0], pos[1], pos[2]);
    extr_data.extr_pos = static_cast<double*>(sol.data[index].rr);
    //printf("stat %d\n", sol.data[index].stat);
    return sol.data[index].stat == SOLQ_FIX;
        
}
void save_obs_dirty(extrpol_t &extr_data, obsd_t *obs, int n_obs) {
    extr_data.sat2prev_obs_dirty.clear();
    
    for (int i = 0; i < n_obs; i++) {
        extr_data.sat2prev_obs_dirty.insert(std::make_pair(obs[i].sat-1, obs[i]));
    }

}
void copy_obs(obsd_t *s_obs, obsd_t *d_obs, int n_obs) {
    for (int i = 0; i < n_obs; i++) {
        d_obs[i] = s_obs[i];
    }
}
void save_obs(extrpol_t &extr_data, obsd_t *obs, int n_obs) {
    extr_data.sat2prev_obs.clear();
    
    for (int i = 0; i < n_obs; i++) {
        extr_data.sat2prev_obs.insert(std::make_pair(obs[i].sat-1, obs[i]));
    }

}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void save_sat_pos(extrpol_t &extr_data, obsd_t *obs, int n_obs, double *rs) {
    int sat;
    extr_data.satpos_prev.clear();
    for (int i = 0; i < n_obs; i++) 
    {  
        sat = obs[i].sat - 1;
        extr_data.satpos_prev.emplace(std::make_pair(sat, vector<double>{rs[i*6], 
                                                                         rs[i*6 + 1], 
                                                                         rs[i*6 + 2]}));
    }
}

vector<double> extrapolate_solution(rtk_t *rtk) {
    auto sol_queue = rtk->extr_data.sol_queue;
    vector<double> pos(3);
    auto first_pos = sol_queue.front(); sol_queue.pop();
    auto second_pos = sol_queue.front(); sol_queue.pop();
    //auto third_pos = sol_queue.front(); sol_queue.pop();
    sol_queue.push(first_pos);
    sol_queue.push(second_pos);
    //sol_queue.push(third_pos);
    double sum = 0.0;
    for (int i = 0; i < 3; i++) {
        rtk->extr_data.prev_pos[i] = second_pos[i];
        pos[i] = 2*second_pos[i] - first_pos[i];//first_pos[i] - 3 * second_pos[i] + 3 * third_pos[i];
        rtk->extr_data.extr_pos[i] = pos[i];
        sum += SQR(pos[i] - second_pos[i]);
    }
    // printf("DELTA POS: %f  \n", sqrt(sum));
    return pos;
}

int update_obs(rtk_t *rtk, const nav_t *nav, obsd_t *obs_cur, int n_obs_cur, int n_all, double *cur_pos)
{
    printf("***********BEFORE********\n");
    prcopt_t *opt = &rtk->opt;
    gtime_t time=obs_cur[0].time;
    auto& extr_data = rtk->extr_data;
    double *rs_cur,*dts,*var,*y,e[3],*azel,*v,*H,*R,*xp,*Pp,*xa,*bias,dt,azeld[2*MAXSAT];
    int i,j,f, ns,ny,nv,sat[MAXSAT],iu[MAXSAT],ir[MAXSAT],niter,prn, k=0;
    int info,vflg[MAXOBS*NFREQ*2+1],svh[MAXOBS*2];
    int nf=opt->ionoopt==IONOOPT_IFLC?1:opt->nf;
    obsd_t obs_prev;
    obsd_t final_obs[2*MAXOBS];
    double range_prev, range_cur, delta;
    /* define local matrices, n=total observations, base + rover */
    rs_cur=mat(6,n_obs_cur);            /* range to satellites */
    dts=mat(2,n_obs_cur);           /* satellite clock biases */
    var=mat(1,n_obs_cur);
    y=mat(nf*2,n_obs_cur);
    azel=zeros(2,n_obs_cur);        /* [az, el] */
    /* compute satellite positions, velocities and clocks */
    satposs(time, obs_cur, n_obs_cur, nav, opt->sateph, rs_cur, dts, var, svh);
    free(dts); free(y); free(var); free(azel);
    int counter_pr = 0, counter_ph = 0;
    extr_data.sat2cur_obs.clear();
    extr_data.sat2range_cur.clear();
    for (int i = 0; i < n_obs_cur; i++) {
        auto sat = obs_cur[i].sat - 1;
        extr_data.sats.erase(sat);
        try {
            obs_prev = extr_data.sat2prev_obs.at(sat);
            range_prev = extr_data.sat2range_prev.at(sat);
        }
        catch(const std::out_of_range::exception& e) {
            final_obs[k++] = obs_cur[i];
            continue;
        }
        
        if ((range_cur=geodist(rs_cur+i*6,cur_pos,e))<=0.0)
            continue;
        
        delta = range_cur - range_prev;
        
        auto lam =  nav->lam[sat][0];
        auto delta_time = obs_cur[i].time.time - obs_prev.time.time + (obs_cur[i].time.sec - obs_prev.time.sec);
        //printf("%f\n", delta);
        if (obs_prev.P[0] > 0.0 && obs_prev.L[0] > 0.0 && !obs_prev.LLI[0]&1) {
            //counter++;
            char id[4] ={0};
            satno2id(sat, id);
            printf("SAT %s PR1: %f PR2: %f DIFF RANGE: %f delta %f\n", id, obs_cur[i].P[0], obs_prev.P[0],obs_cur[i].P[0] - obs_prev.P[0] - delta, delta);
            if (obs_cur[i].P[0] > 0 && abs(obs_cur[i].P[0] - obs_prev.P[0] - delta) > PR_OUTLIER) {
                counter_pr++;
                obs_cur[i].P[0] = obs_prev.P[0] + delta;
            }
            
            // printf("SAT %s PH1: %f PH2: %f DIFF PHASE: %f delta/lam %f   %f   \n", id,obs_cur[i].L[0], obs_prev.L[0], obs_cur[i].L[0] - obs_prev.L[0] - delta/lam , delta/lam, PH_OUTLIER/lam);
            // if (obs_cur[i].L[0] > 0 && abs(obs_cur[i].L[0] - obs_prev.L[0] - delta/lam ) > PH_OUTLIER/lam) {
            //     counter_ph++;
            //     obs_cur[i].L[0] = obs_prev.L[0] + delta/lam ;
            // }
            //obs_cur[i].SNR[0] = obs_prev.SNR[0];
            //
        }
        final_obs[k++] = obs_cur[i];
    }
    printf("BEFORE PR :%d PH: %d    \n", counter_pr, counter_ph);
    int count = 0;
    // for (auto sat: extr_data.sats) {
    //     if (count > 3) {
    //         break;
    //     }
    //     count++;
    //     try {
    //         obs_prev = extr_data.sat2prev_obs.at(sat);
    //         range_prev = extr_data.sat2range_prev.at(sat);
    //     }
    //     catch(const std::out_of_range::exception& e) {
    //         continue;
    //     }
    //     double *rs = mat(6, 1);
    //     satposs(time, &obs_prev, 1, nav, opt->sateph, rs, dts, var, svh);
    //     if ((range_cur=geodist(rs,cur_pos,e))<=0.0)
    //         continue;
    //     delta = range_cur - range_prev;
    
    //     auto lam =  nav->lam[sat][0];
    //     if (obs_prev.P[0] > 0.0 && obs_prev.L[0] > 0.0 && !obs_prev.LLI[0]&1) {
    //         final_obs[k] = obs_cur[0];
    //         final_obs[k].P[0] = obs_prev.P[0] + delta;
    //         final_obs[k].L[0] = obs_prev.L[0] + delta/lam;
    //         // obs_cur[i].SNR[0] = obs_prev.SNR[0];
    //         //final_obs[k].LLI[0] = 0;
    //         k++;
    //     }
        
    // }

    for (int i = n_obs_cur; i < n_all; i++) {
        final_obs[k++] = obs_cur[i];
    }

    for (int i = 0; i < k; i++) {
        obs_cur[i] = final_obs[i];
    }
    free(rs_cur);
    return k;
}

void save_ranges(rtk_t *rtk, obsd_t *obs, int n_obs, const nav_t *nav, double *pos) {
    prcopt_t *opt = &rtk->opt;
    gtime_t time=obs[0].time;
    auto& extr_data = rtk->extr_data;
    double *rs_cur,*dts,*var,*y,e[3],*azel,*v,*H,*R,*xp,*Pp,*xa,*bias,dt,azeld[2*MAXSAT];
    int i,j,f, ns,ny,nv,sat[MAXSAT],iu[MAXSAT],ir[MAXSAT],niter,prn;
    int info,vflg[MAXOBS*NFREQ*2+1],svh[MAXOBS*2];
    int nf=opt->ionoopt==IONOOPT_IFLC?1:opt->nf;
    obsd_t obs_prev;
    double cur_pos[3];
    double range_prev, range_cur, delta;
    // if (extr_data.sol_queue.size() == 0) return;
    // auto& pos = extr_data.sol_queue.front();
    for (int i=0;i<3;i++) {
        cur_pos[i] = pos[i];
    }
    /* define local matrices, n=total observations, base + rover */
    extr_data.sats.clear();
    rs_cur=mat(6,n_obs);            /* range to satellites */
    dts=mat(2,n_obs);           /* satellite clock biases */
    var=mat(1,n_obs);
    y=mat(nf*2,n_obs);
    azel=zeros(2,n_obs);        /* [az, el] */
    /* compute satellite positions, velocities and clocks */
    satposs(time, obs, n_obs, nav, opt->sateph, rs_cur, dts, var, svh);
    extr_data.sat2range_prev.clear();
    extr_data.sat2prev_obs.clear();
    free(dts); free(y); free(var); free(azel);
    printf("WTF******************");
    for (int i = 0; i < n_obs; i++) {
        extr_data.sats.insert(obs[i].sat - 1);
        if ((range_cur=geodist(rs_cur+i*6,cur_pos,e))<=0.0)
            continue;
        extr_data.sat2range_prev[obs[i].sat-1] = range_cur;
        extr_data.sat2prev_obs[obs[i].sat-1] = obs[i];
    }
    free(rs_cur);
}

int restore_missed_sats(rtk_t *rtk, obsd_t *obs, int n_obs, const nav_t *nav, int n_all)
{
    prcopt_t *opt = &rtk->opt;
    gtime_t time=obs[0].time;
    static bool flag = false;
    double *rs,*dts,*var,*y,e[3],*azel,*v,*H,*R,*xp,*Pp,*xa,*bias,dt,azeld[2*MAXSAT];
    int i,j,f, n,ny,nv,sat[MAXSAT],iu[MAXSAT],ir[MAXSAT],niter,prn;
    int info,vflg[MAXOBS*NFREQ*2+1],svh[MAXOBS*2];
    int stat=rtk->opt.mode<=PMODE_DGPS?SOLQ_DGPS:SOLQ_FLOAT;
    int nf=opt->ionoopt==IONOOPT_IFLC?1:opt->nf;
    
    obsd_t copied_obs[MAXOBS], copied_obs_base[MAXOBS];
    if (rtk->extr_data.sol_queue.size() == 1) {
        //auto pos = extrapolate_solution(rtk);
        //double rr_pos[3] = {pos[0], pos[1], pos[2]};
        auto stat = get_sol_from_pos(rtk->extr_data, obs[0].time);
        //save_ranges(rtk, obs, n_obs, nav, rtk->extr_data.prev_pos);
        //auto res = update_obs(rtk, nav, obs, n_obs, n_all, rr_pos);
        //printf("%d \n", stat);
        if (!stat) {
            std::queue<std::vector<double>> empty{};
            std::swap(rtk->extr_data.sol_queue, empty);
        }
        else {
            auto res = update_obs(rtk, nav, obs, n_obs, n_all, rtk->extr_data.extr_pos);
        }
        return n_all;
    }
    else {
        if (rtk->extr_data.prev_pos) {
            // auto pos = rtk->extr_data.sol_queue.front();
            // double rr_pos[3];
            // for (int i = 0; i < 3; i++) {
            //     rr_pos[i]= pos[i];
            // }
            save_ranges(rtk, obs, n_obs, nav, rtk->extr_data.prev_pos);
        }
        return n_all;
    }
}

void post_update_obs(rtk_t *rtk, const nav_t *nav, obsd_t *obs_cur, int n_obs_cur, double *cur_pos)
{
    printf("********AFTER************\n");
    prcopt_t *opt = &rtk->opt;
    gtime_t time=obs_cur[0].time;
    auto& extr_data = rtk->extr_data;
    int i,j,f, ns,ny,nv,sat[MAXSAT],iu[MAXSAT],ir[MAXSAT],niter,prn;
    double *rs_cur,*dts,*var,*y,e[3],*azel,*v,*H,*R,*xp,*Pp,*xa,*bias,dt,azeld[2*MAXSAT];

    int info,vflg[MAXOBS*NFREQ*2+1],svh[MAXOBS*2];
    int nf=opt->ionoopt==IONOOPT_IFLC?1:opt->nf;
    obsd_t obs_prev;
    
    double range_prev, range_cur, delta;
    /* define local matrices, n=total observations, base + rover */
   
    rs_cur=mat(6,n_obs_cur);            /* range to satellites */
    dts=mat(2,n_obs_cur);           /* satellite clock biases */
    var=mat(1,n_obs_cur);
    y=mat(nf*2,n_obs_cur);
    azel=zeros(2,n_obs_cur);        /* [az, el] */
    /* compute satellite positions, velocities and clocks */
    satposs(time, obs_cur, n_obs_cur, nav, opt->sateph, rs_cur, dts, var, svh);
    extr_data.sats.clear();
    extr_data.sat2cur_obs.clear();
    extr_data.sat2range_cur.clear();
    // extr_data.sat2range_prev.clear();
    // extr_data.sat2prev_obs.clear();
    free(dts); free(y); free(var); free(azel);
    int counter_pr = 0, counter_ph = 0;
    for (int i = 0; i < n_obs_cur; i++) {
        auto sat = obs_cur[i].sat - 1;
        try {
            obs_prev = extr_data.sat2prev_obs.at(sat);
            range_prev = extr_data.sat2range_prev.at(sat);
        }
        catch(const std::out_of_range::exception& e) {
            extr_data.sat2cur_obs[sat] = obs_cur[i];
            continue;
        }
        
        if ((range_cur=geodist(rs_cur+i*6,cur_pos,e))<=0.0)
            continue;
        delta = range_cur - range_prev;
        
        extr_data.sat2range_cur[sat] = range_cur;
        extr_data.sats.insert(sat);
        auto lam =  nav->lam[sat][0];
       // printf("LAM %f\n", lam);
        auto delta_time = obs_cur[i].time.time - obs_prev.time.time + (obs_cur[i].time.sec - obs_prev.time.sec);
        //printf("DELTA: %f DELTA TIME %f\n", delta, delta_time);
        if (obs_prev.P[0] > 0.0 && obs_prev.L[0] > 0.0 && !obs_prev.LLI[0]&1) {
            //counter++;
            char id[4] ={0};
            satno2id(sat, id);
            printf("SAT %s PR1: %f PR2: %f DIFF RANGE: %f delta %f\n", id, obs_cur[i].P[0], obs_prev.P[0],obs_cur[i].P[0] - obs_prev.P[0] - delta, delta);
            if (obs_cur[i].P[0] > 0 && abs(obs_cur[i].P[0] - obs_prev.P[0] - delta) > PR_OUTLIER) {
                counter_pr++;
                obs_cur[i].P[0] = obs_prev.P[0] + delta;
            }
            
            // printf("SAT %s PH1: %f PH2: %f DIFF PHASE: %f delta/lam %f   %f   \n", id,obs_cur[i].L[0], obs_prev.L[0], obs_cur[i].L[0] - obs_prev.L[0] - delta/lam , delta/lam, PH_OUTLIER/lam);
            // if (obs_cur[i].L[0] > 0 && abs(obs_cur[i].L[0] - obs_prev.L[0] - delta/lam ) > PH_OUTLIER/lam) {
            //     counter_ph++;
            //     obs_cur[i].L[0] = obs_prev.L[0] + delta/lam ;
            // }
            //obs_cur[i].SNR[0] = obs_prev.SNR[0];
            
        }
        extr_data.sat2cur_obs[sat] = obs_cur[i];
        // else {
        //     auto obs_prev = extr_data.sat2prev_obs_dirty.at(sat);
        //     if (obs_prev.P[0] > 0.0 && obs_prev.L[0] > 0.0 && !obs_prev.LLI[0]&1 &&!obs_prev.LLI[0]&2) {
        //         obs_cur[i].P[0] = obs_prev.P[0] + delta;
        //         obs_cur[i].L[0] = obs_prev.L[0] + delta/lam;
        //         obs_cur[i].SNR[0] = obs_prev.SNR[0];
        //         obs_cur[i].LLI[0] = 0;
        //     }
        // }
    }
    std::swap(extr_data.sat2prev_obs, extr_data.sat2prev_obs);
    std::swap(extr_data.sat2range_prev, extr_data.sat2range_cur);
    printf("AFTER PR :%d PH: %d  \n", counter_pr, counter_ph);

    save_obs(extr_data, obs_cur, n_obs_cur);
    free(rs_cur);
}

