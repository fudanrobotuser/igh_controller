/**
 * 基于 ethercat igh example dc_user 例子改写的 伺服控制器
 * 功能为:
 *  1 igh 主站配置
 *  2 伺服状态机切换
 *  3 伺服位置写入
 *  4 伺服状态反馈
 * @date 20240609
 * @author wei1224hf@gmail.com
 * @version beta.1
 * ****************************/
#include "ecrt.h"
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/types.h>
#include <malloc.h>
#include <inttypes.h>
#include <math.h>  // 包含数学库

#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include "ecat_data_buffer.h"

#include <pthread.h>
#include <unistd.h>

#define THREAD_PRIORITY 60 // Priority for the real-time thread
#define PI 3.1415926

// 共享内存区域,2M大小
#define SHM_KEY 12345
#define SHM_SIZE (1024 * 1024 * 2)
bool dataOk = false;

static unsigned int counter = 0;
static unsigned int sync_ref_counter = 0;
static struct timespec apptime;

// 同步周期时钟配置,8毫秒下发周期控制
#define TASK_FREQUENCY 500 /* Hz */
#define CLOCK_TO_USE CLOCK_REALTIME
#define TIMEOUT_CLEAR_ERROR (1 * TASK_FREQUENCY) /* clearing error timeout */
// 时间控制相关函数
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / TASK_FREQUENCY)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
const struct timespec cycletime = {0, PERIOD_NS};

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

int ii = 0;
bool toZero = true;
int toZeroOffset = 200;

GROUP_FEEDBACK feedback;
GROUP_REFERENCE reference;

/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL;

// elmo 伺服对接
#define ElmoVidPid 0x0000009a, 0x00030924

#define KaiXuanVidPid 0x00010203, 0x00000402

// offsets for PDO entries
static unsigned int ctrl_word;
static unsigned int mode;
static unsigned int tar_torq;
static unsigned int max_torq;
static unsigned int tar_pos;

static unsigned int digital_output;
static unsigned int max_speed;
static unsigned int touch_probe_func;
static unsigned int tar_vel;
static unsigned int error_code;
static unsigned int status_word;
static unsigned int mode_display;
static unsigned int pos_act;
static unsigned int vel_act;
static unsigned int torq_act;
static unsigned int pos_gap_act;
static unsigned int touch_probe_status;
static unsigned int touch_probe_pos;
static unsigned int touch_probe_pos2;
static unsigned int digital_input;

//default postions
int defaultPositions[33] = {0, 0, 0, 0, 0, 0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0};


// 判断是否所有电机都到了零位,到了之后会变成 true
bool isInitedToDefault[33] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,  false, false, false, false, false,  false, false, false, false};
// 是否电机使能了
bool isEnabled[33] = {false, false, false, false, false,  false, false, false, false, false,  false, false, false, false, false, false, false, false, false, false, false, false, false, false,  false, false, false, false, false,  false, false, false, false};
// 记录每个电机的旧状态
uint16_t statusOld[33] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0};
// 状态机切换的等待次数
int statusDeCount[33] = {5, 5, 5, 5, 5,  5, 5, 5, 5, 5,  5, 5, 5, 5, 5,  5, 5, 5, 5, 5, 5, 5, 5, 5,  5, 5, 5, 5, 5,  5, 5, 5, 5};
// 记录当前的位置
int last_position[33] = {0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0,  0, 0, 0, 0};

// 伺服电机所有属性结构体,用于读取值的位置指针
static struct
{
    unsigned int ctrl_word;
    unsigned int target_position;
    unsigned int target_torque;
    unsigned int target_velocity;
    unsigned int max_torque;
    unsigned int DO;
    unsigned int DI;
    unsigned int act_DI;

    unsigned int offset_velocity;
    unsigned int offset_torque;
    unsigned int offset_position;

    unsigned int error_code;
    unsigned int status_word;
    unsigned int act_position;
    unsigned int act_torque;

    unsigned int disp_mode;
    unsigned int actual_ferr;
    unsigned int probe_status;
    unsigned int probe1_pos;

    unsigned int probe2_pos;
    unsigned int act_velocity;
    unsigned int mode_Of_Operation;
    unsigned int mode_Of_Operation_dsiplay;
} offset[33];

// IGH主栈的主映射表,总计15个电机. 因为 0-3 口给扩展板用了,实际电机的总线分配序号为 4-18
const static ec_pdo_entry_reg_t domain1_regs[] = {

{0, 0, KaiXuanVidPid, 0x6040, 0, &offset[0].ctrl_word},
{0, 0, KaiXuanVidPid, 0x6071, 0, &offset[0].target_torque},
{0, 0, KaiXuanVidPid, 0x607a, 0, &offset[0].target_position},
{0, 0, KaiXuanVidPid, 0x60b1, 0, &offset[0].offset_velocity},
{0, 0, KaiXuanVidPid, 0x60b2, 0, &offset[0].offset_torque},
{0, 0, KaiXuanVidPid, 0x60ff, 0, &offset[0].target_velocity},

{0, 0, KaiXuanVidPid, 0x6041, 0, &offset[0].status_word},
{0, 0, KaiXuanVidPid, 0x6064, 0, &offset[0].act_position},
{0, 0, KaiXuanVidPid, 0x606c, 0, &offset[0].act_velocity},
{0, 0, KaiXuanVidPid, 0x6077, 0, &offset[0].act_torque},
{0, 0, KaiXuanVidPid, 0x6061, 0, &offset[0].mode_Of_Operation_dsiplay},

{0, 1, KaiXuanVidPid, 0x6040, 0, &offset[1].ctrl_word},
{0, 1, KaiXuanVidPid, 0x6071, 0, &offset[1].target_torque},
{0, 1, KaiXuanVidPid, 0x607a, 0, &offset[1].target_position},
{0, 1, KaiXuanVidPid, 0x60b1, 0, &offset[1].offset_velocity},
{0, 1, KaiXuanVidPid, 0x60b2, 0, &offset[1].offset_torque},
{0, 1, KaiXuanVidPid, 0x60ff, 0, &offset[1].target_velocity},

{0, 1, KaiXuanVidPid, 0x6041, 0, &offset[1].status_word},
{0, 1, KaiXuanVidPid, 0x6064, 0, &offset[1].act_position},
{0, 1, KaiXuanVidPid, 0x606c, 0, &offset[1].act_velocity},
{0, 1, KaiXuanVidPid, 0x6077, 0, &offset[1].act_torque},
{0, 1, KaiXuanVidPid, 0x6061, 0, &offset[1].mode_Of_Operation_dsiplay},

{0, 2, KaiXuanVidPid, 0x6040, 0, &offset[2].ctrl_word},
{0, 2, KaiXuanVidPid, 0x6071, 0, &offset[2].target_torque},
{0, 2, KaiXuanVidPid, 0x607a, 0, &offset[2].target_position},
{0, 2, KaiXuanVidPid, 0x60b1, 0, &offset[2].offset_velocity},
{0, 2, KaiXuanVidPid, 0x60b2, 0, &offset[2].offset_torque},
{0, 2, KaiXuanVidPid, 0x60ff, 0, &offset[2].target_velocity},

{0, 2, KaiXuanVidPid, 0x6041, 0, &offset[2].status_word},
{0, 2, KaiXuanVidPid, 0x6064, 0, &offset[2].act_position},
{0, 2, KaiXuanVidPid, 0x606c, 0, &offset[2].act_velocity},
{0, 2, KaiXuanVidPid, 0x6077, 0, &offset[2].act_torque},
{0, 2, KaiXuanVidPid, 0x6061, 0, &offset[2].mode_Of_Operation_dsiplay},

{0, 3, KaiXuanVidPid, 0x6040, 0, &offset[3].ctrl_word},
{0, 3, KaiXuanVidPid, 0x6071, 0, &offset[3].target_torque},
{0, 3, KaiXuanVidPid, 0x607a, 0, &offset[3].target_position},
{0, 3, KaiXuanVidPid, 0x60b1, 0, &offset[3].offset_velocity},
{0, 3, KaiXuanVidPid, 0x60b2, 0, &offset[3].offset_torque},
{0, 3, KaiXuanVidPid, 0x60ff, 0, &offset[3].target_velocity},

{0, 3, KaiXuanVidPid, 0x6041, 0, &offset[3].status_word},
{0, 3, KaiXuanVidPid, 0x6064, 0, &offset[3].act_position},
{0, 3, KaiXuanVidPid, 0x606c, 0, &offset[3].act_velocity},
{0, 3, KaiXuanVidPid, 0x6077, 0, &offset[3].act_torque},
{0, 3, KaiXuanVidPid, 0x6061, 0, &offset[3].mode_Of_Operation_dsiplay},

{0, 4, KaiXuanVidPid, 0x6040, 0, &offset[4].ctrl_word},
{0, 4, KaiXuanVidPid, 0x6071, 0, &offset[4].target_torque},
{0, 4, KaiXuanVidPid, 0x607a, 0, &offset[4].target_position},
{0, 4, KaiXuanVidPid, 0x60b1, 0, &offset[4].offset_velocity},
{0, 4, KaiXuanVidPid, 0x60b2, 0, &offset[4].offset_torque},
{0, 4, KaiXuanVidPid, 0x60ff, 0, &offset[4].target_velocity},

{0, 4, KaiXuanVidPid, 0x6041, 0, &offset[4].status_word},
{0, 4, KaiXuanVidPid, 0x6064, 0, &offset[4].act_position},
{0, 4, KaiXuanVidPid, 0x606c, 0, &offset[4].act_velocity},
{0, 4, KaiXuanVidPid, 0x6077, 0, &offset[4].act_torque},
{0, 4, KaiXuanVidPid, 0x6061, 0, &offset[4].mode_Of_Operation_dsiplay},

{0, 5, KaiXuanVidPid, 0x6040, 0, &offset[5].ctrl_word},
{0, 5, KaiXuanVidPid, 0x6071, 0, &offset[5].target_torque},
{0, 5, KaiXuanVidPid, 0x607a, 0, &offset[5].target_position},
{0, 5, KaiXuanVidPid, 0x60b1, 0, &offset[5].offset_velocity},
{0, 5, KaiXuanVidPid, 0x60b2, 0, &offset[5].offset_torque},
{0, 5, KaiXuanVidPid, 0x60ff, 0, &offset[5].target_velocity},

{0, 5, KaiXuanVidPid, 0x6041, 0, &offset[5].status_word},
{0, 5, KaiXuanVidPid, 0x6064, 0, &offset[5].act_position},
{0, 5, KaiXuanVidPid, 0x606c, 0, &offset[5].act_velocity},
{0, 5, KaiXuanVidPid, 0x6077, 0, &offset[5].act_torque},
{0, 5, KaiXuanVidPid, 0x6061, 0, &offset[5].mode_Of_Operation_dsiplay},

{0, 6, KaiXuanVidPid, 0x6040, 0, &offset[6].ctrl_word},
{0, 6, KaiXuanVidPid, 0x6071, 0, &offset[6].target_torque},
{0, 6, KaiXuanVidPid, 0x607a, 0, &offset[6].target_position},
{0, 6, KaiXuanVidPid, 0x60b1, 0, &offset[6].offset_velocity},
{0, 6, KaiXuanVidPid, 0x60b2, 0, &offset[6].offset_torque},
{0, 6, KaiXuanVidPid, 0x60ff, 0, &offset[6].target_velocity},

{0, 6, KaiXuanVidPid, 0x6041, 0, &offset[6].status_word},
{0, 6, KaiXuanVidPid, 0x6064, 0, &offset[6].act_position},
{0, 6, KaiXuanVidPid, 0x606c, 0, &offset[6].act_velocity},
{0, 6, KaiXuanVidPid, 0x6077, 0, &offset[6].act_torque},
{0, 6, KaiXuanVidPid, 0x6061, 0, &offset[6].mode_Of_Operation_dsiplay},
    ////
    {}};

// 伺服电机的PDO映射参数
ec_pdo_entry_info_t Igh_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6071, 0x00, 16},
    {0x607a, 0x00, 32},
    {0x60b1, 0x00, 32},
    {0x60b2, 0x00, 16},
    {0x60ff, 0x00, 32},

    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x6061, 0x00, 8},
};

// 伺服电机PDO映射参数的组地址
ec_pdo_info_t Igh_pdos[] = {
    {0x1607, 6, Igh_pdo_entries + 0},
    {0x1a07, 5, Igh_pdo_entries + 6},
};

ec_sync_info_t Igh_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, Igh_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, Igh_pdos + 1, EC_WD_DISABLE},
    {0xFF}};

ec_pdo_info_t Igh_pdos_kaixuan[] = {
    {0x1601, 6, Igh_pdo_entries + 0},
    {0x1a01, 5, Igh_pdo_entries + 6},
};

ec_sync_info_t Igh_syncs_kaixuan[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, Igh_pdos_kaixuan + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, Igh_pdos_kaixuan + 1, EC_WD_DISABLE},
    {0xFF}};    

/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/**
 * 主时钟循环程序,要每8毫秒执行一次,
 * 功能为,切换电机的状态机,读取上层应用下发的位置命令,写入电机的下发位置,
 * 读取电机当前的 位置 速度 扭力 ，反馈给上层应用
 * **/
void *rt_thread_function(void *arg)
{
    struct timespec wakeupTime, time;
    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);
    int act_position = 0;
    int act_torque = 0;
    int act_velocity = 0;
    int target_postion = 0;
    int target_torque_offset = 0;
    int act_positin_read = 0;
    int dir = 1;
    bool isAllEnabled = true;
    bool isAllInitedToZero = true;
    int i2, i3, i4 = 0;
    bool pushDataOK = false;
    bool pullDataOK = false;
    while (1)
    {

        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);


        pullDataOK = false;
        isAllEnabled = true;
        isAllInitedToZero = true;

        for (i2 = 0; i2 <= 6; i2++)
        {
            uint16_t ss = EC_READ_U16(domain1_pd + offset[i2].status_word);
            if (statusOld[i2] != ss)
            {
                if ((ss==0x1237&&statusOld[i2]==0x0237)||(ss==0x0237&&statusOld[i2]==0x1237))
                {
                    //NO PRINT
                }
                else
                {
                    printf("status %d : 0x%04x to 0x%04x  \n", i2, statusOld[i2], ss);
                }
                
                statusOld[i2] = ss;
                statusDeCount[i2] = 5;
            }

            // 电机未使能,则执行状态机切换
            if ((ss & 0xFF) != 0x37)
            {
                isEnabled[i2] = false;
                if (statusDeCount[i2] == 5)
                {
                    if ((ss & 0xFF) == 0x50)
                    {
                        EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x06);
                    }
                    else if ((ss & 0xFF) == 0x31)
                    {
                        last_position[i2] = EC_READ_S32(domain1_pd + offset[i2].act_position);
                        EC_WRITE_S32(domain1_pd + offset[i2].target_position, last_position[i2]);
                        EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x07);
                    }
                    else if ((ss & 0xFF) == 0x33)
                    {
                        last_position[i2] = EC_READ_S32(domain1_pd + offset[i2].act_position);
                        EC_WRITE_S32(domain1_pd + offset[i2].target_position, last_position[i2]);
                        EC_WRITE_U16(domain1_pd + offset[i2].ctrl_word, 0x0F);
                    }
                }
                if (statusDeCount[i2] <= 0)
                {
                    statusDeCount[i2] = 5;
                }
                else
                {
                    statusDeCount[i2]--;
                }
            }
            else
            {
                isEnabled[i2] = true;

                // 判断所有电机都使能
                for (i3 = 0; i3 <= 6; i3++)
                {
                    isAllEnabled = (isAllEnabled && isEnabled[i3]);
                }
                if (isAllEnabled)
                {
                    act_position = EC_READ_S32(domain1_pd + offset[i2].act_position);
                    act_velocity = EC_READ_S32(domain1_pd + offset[i2].act_velocity);
                    act_torque = EC_READ_S16(domain1_pd + offset[i2].act_torque);

                    if (isInitedToDefault[i2] == false)
                    {
                        if (last_position[i2] > defaultPositions[i2] + 8000)
                        {
                            target_postion = last_position[i2] - 12000;
                            printf("position + %d,  %d to %d  \n", i2, last_position[i2], target_postion);
                            EC_WRITE_S32(domain1_pd + offset[i2].target_position, target_postion);
                            last_position[i2] = target_postion;
                        }
                        else if (last_position[i2] < defaultPositions[i2] - 8000)
                        {
                            target_postion = last_position[i2] + 12000;
                            printf("position - %d,  %d to %d  \n", i2, last_position[i2], target_postion);
                            EC_WRITE_S32(domain1_pd + offset[i2].target_position, target_postion);
                            last_position[i2] = target_postion;                            
                        }
                        else
                        {
                            isInitedToDefault[i2] = true;
                            //EC_WRITE_S16(domain1_pd + offset[i2].offset_torque, 100);
                            //printf("offset_torque 10 \n");
                        }
                    }

                   
                }
            }
        }

        Igh_rechekTime();
    }
    return NULL;
}

void Igh_rechekTime()
{
    clock_gettime(CLOCK_TO_USE, &apptime);

    ecrt_master_application_time(master, TIMESPEC2NS(apptime));
    if (sync_ref_counter)
    {
        sync_ref_counter--;
    }
    else
    {
        sync_ref_counter = 1;                     // sync every cycle
        ecrt_master_sync_reference_clock(master); // DC reference clock drift compensation
    }
    ecrt_master_sync_slave_clocks(master); // DC clock drift compensation
    // queue process data
    ecrt_domain_queue(domain1);
    // send EtherCAT data
    ecrt_master_send(master);
}

void Igh_master_activate()
{
    printf("......Activating master......\n");
    if (ecrt_master_activate(master))
    {
        exit(EXIT_FAILURE);
    }
    if (!(domain1_pd = ecrt_domain_data(domain1)))
    {
        exit(EXIT_FAILURE);
    }

    printf("......Master  Activated.......\n");
}

/****************************************************************************/

void Igh_init()
{
    ec_master_info_t master_info;
    ec_slave_info_t slave_info;
    int ret;
    int slavecnt;

    // uint32_t  abort_code;
    // size_t rb;
    int i = 0;
    master = ecrt_request_master(0);
    if (!master)
    {
        exit(EXIT_FAILURE);
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        exit(EXIT_FAILURE);
    }

    //---------get master / salve info----------------------
    ret = ecrt_master(master, &master_info);
    slavecnt = master_info.slave_count;
    printf("ret = %d, slavecnt = %d, apptime = %" PRIu64 "\n", ret, master_info.slave_count, master_info.app_time);
    ret = ecrt_master_get_slave(master, 0, &slave_info);
    printf("ret = %d,spos = %d, pcode = %d\n", ret, slave_info.position, slave_info.product_code);

    //---------end get master / salve info----------------------

    printf("servo %d  begin init! \n", i);



    for (ii = 0; ii <= 6; ii++)
    {

        ec_slave_config_t *sc;
        if (!(sc = ecrt_master_slave_config(master, 0, ii, KaiXuanVidPid)))
        {
            fprintf(stderr, "Failed to get slave configuration for Igh.\n");
            exit(EXIT_FAILURE);
        }

        printf("Configuring PDOs... %d \n", ii);
        if (ecrt_slave_config_pdos(sc, EC_END, Igh_syncs_kaixuan))
        {
            fprintf(stderr, "Failed to configure Igh PDOs.\n");
            exit(EXIT_FAILURE);
        }

        ecrt_slave_config_sdo16(sc, 0x1C32, 1, 2); // set output synchronization triggered by  sync0 event DC mode
        ecrt_slave_config_sdo16(sc, 0x1C33, 1, 2); // set input  synchronization triggered by  sync1 evnent DC mode
        ecrt_slave_config_sdo8(sc, 0x6060, 0, 8);
        ecrt_slave_config_dc(sc, 0x0300, PERIOD_NS, PERIOD_NS/2, 0, 0);
    }    

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv)
{
   

   

    Igh_init();
    Igh_master_activate();

    pthread_t rt_thread;
    pthread_attr_t attr;
    struct sched_param param;

    // Initialize thread attributes
    if (pthread_attr_init(&attr) != 0)
    {
        perror("pthread_attr_init");
        exit(EXIT_FAILURE);
    }

    // Set scheduling policy to SCHED_FIFO
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO) != 0)
    {
        perror("pthread_attr_setschedpolicy");
        exit(EXIT_FAILURE);
    }

    // Set thread priority
    param.sched_priority = THREAD_PRIORITY;
    if (pthread_attr_setschedparam(&attr, &param) != 0)
    {
        perror("pthread_attr_setschedparam");
        exit(EXIT_FAILURE);
    }

    // Make the scheduling parameters take effect
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0)
    {
        perror("pthread_attr_setinheritsched");
        exit(EXIT_FAILURE);
    }

    // Create real-time thread
    if (pthread_create(&rt_thread, &attr, rt_thread_function, NULL) != 0)
    {
        printf("pthread_create\n");
        exit(EXIT_FAILURE);
    }

    // Wait for the thread to complete
    if (pthread_join(rt_thread, NULL) != 0)
    {
        perror("pthread_join");
        exit(EXIT_FAILURE);
    }

    // Destroy thread attributes
    if (pthread_attr_destroy(&attr) != 0)
    {
        perror("pthread_attr_destroy");
        exit(EXIT_FAILURE);
    }

    return 0;
}

/****************************************************************************/