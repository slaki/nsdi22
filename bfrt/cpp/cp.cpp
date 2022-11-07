#include <bf_rt/bf_rt_info.hpp>
#include <bf_rt/bf_rt_init.hpp>
#include <bf_rt/bf_rt_common.h>
#include <bf_rt/bf_rt_table_key.hpp>
#include <bf_rt/bf_rt_table_data.hpp>
#include <bf_rt/bf_rt_table.hpp>
#include <getopt.h>
#include <iostream>
//#include <chrono>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <bf_switchd/bf_switchd.h>
#ifdef __cplusplus
}
#endif

/***********************************************************************************
 * This sample cpp application code is based on the P4 program
 *tna_exact_match.p4
 * Please refer to the P4 program and the generated bf-rt.json for information
 *on
 * the tables contained in the P4 program, and the associated key and data
 *fields.
 **********************************************************************************/

#define robot_id_t uint8_t
#define bunny_id_t uint16_t
#define joint_id_t uint8_t
#define p4_time_t uint32_t
#define dec_t uint64_t

namespace bfrt {
namespace examples {
namespace tna_exact_match {

struct bunny_key_t
{
    robot_id_t robot_id;
    bunny_id_t actual_bunny;
    joint_id_t jointId;
};

struct bunny_data_t
{
    bunny_id_t next_id;
    p4_time_t duration;
    /*action set_target(BUNNY_ID_T next_id,
                      TIME_T duration){...} */
};


struct bunny_target_t
{
    dec_t tpos;
    dec_t tspeed;
};


// Structure definition to represent the key of the ipRoute table
struct IpRouteKey {
  uint32_t ipDstAddr;
  uint16_t vrf;
};

// Structure definition to represent the data of the ipRoute table for action
// "route"
struct IpRoute_routeData {
  uint64_t srcMac;
  uint64_t dstMac;
  uint16_t dst_port;
};

// Structure definition to represent the data of the ipRoute table for action
// "nat"
struct IpRoute_natData {
  uint32_t srcAddr;
  uint32_t dstAddr;
  uint16_t dst_port;
};

// Structure definition tp represent the data of the ipRoute table
struct IpRouteData {
  union {
    IpRoute_routeData route_data;
    IpRoute_natData nat_data;
  } data;
  // Based on the action_id, contents of the enum are interpreted
  bf_rt_id_t action_id;
};

namespace {
// Key field ids, table data field ids, action ids, Table object required for
// interacting with the table
const bfrt::BfRtInfo *bfrtInfo = nullptr;
    const bfrt::BfRtTable *ipRouteTable = nullptr;
const bfrt::BfRtTable *iBunnyTable = nullptr;
const bfrt::BfRtTable *eBunnyTable = nullptr;
std::shared_ptr<bfrt::BfRtSession> session;

std::unique_ptr<bfrt::BfRtTableKey> bfrtTableKey;
std::unique_ptr<bfrt::BfRtTableData> bfrtTableData;

std::unique_ptr<bfrt::BfRtTableKey> iTableKey;
std::unique_ptr<bfrt::BfRtTableData> iTableData;
std::unique_ptr<bfrt::BfRtTableKey> eTableKey;
std::unique_ptr<bfrt::BfRtTableData> eTableData;


// Key field ids
    bf_rt_id_t ipRoute_ip_dst_field_id = 0;
    bf_rt_id_t ipRoute_vrf_field_id = 0;
bf_rt_id_t i_robot_id_field = 0;
bf_rt_id_t i_actual_bunny_field = 0;
bf_rt_id_t i_joint_id_field = 0;

bf_rt_id_t e_robot_id_field = 0;
bf_rt_id_t e_actual_bunny_field = 0;
bf_rt_id_t e_joint_id_field = 0;

// Action Ids
    bf_rt_id_t ipRoute_route_action_id = 0;
    bf_rt_id_t ipRoute_nat_action_id = 0;
bf_rt_id_t ibunny_set_target_id = 0;
bf_rt_id_t ebunny_set_target_id = 0;

// Data field Ids 
bf_rt_id_t iBunnyTable_next_id = 0;
bf_rt_id_t iBunnyTable_duration = 0;
bf_rt_id_t eBunnyTable_tpos = 0;
bf_rt_id_t eBunnyTable_tspeed = 0;



// Data field Ids 
    bf_rt_id_t ipRoute_route_action_src_mac_field_id = 0;
    bf_rt_id_t ipRoute_route_action_dst_mac_field_id = 0;
    bf_rt_id_t ipRoute_route_action_port_field_id = 0;

// Data field ids 
    bf_rt_id_t ipRoute_nat_action_ip_src_field_id = 0;
    bf_rt_id_t ipRoute_nat_action_ip_dst_field_id = 0;
    bf_rt_id_t ipRoute_nat_action_port_field_id = 0;

#define ALL_PIPES 0xffff
bf_rt_target_t dev_tgt;
}  // anonymous namespace

// This function does the initial setUp of getting bfrtInfo object associated
// with the P4 program from which all other required objects are obtained
void setUp() {
  dev_tgt.dev_id = 0;
  dev_tgt.pipe_id = ALL_PIPES;
  // Get devMgr singleton instance
  auto &devMgr = bfrt::BfRtDevMgr::getInstance();

  // Get bfrtInfo object from dev_id and p4 program name
  auto bf_status =
      devMgr.bfRtInfoGet(dev_tgt.dev_id, "ur", &bfrtInfo);
  // Check for status
  assert(bf_status == BF_SUCCESS);

  // Create a session object
  session = bfrt::BfRtSession::sessionCreate();
}

// This function does the initial set up of getting key field-ids, action-ids
// and data field ids associated with the ipRoute table. This is done once
// during init time.
void tableSetUp() {
  // Get table object from name
        //  auto bf_status =
        //      bfrtInfo->bfrtTableFromNameGet("SwitchIngress.ipRoute", &ipRouteTable);
        //  assert(bf_status == BF_SUCCESS);

  auto bf_status = bfrtInfo->bfrtTableFromNameGet("SwitchIngress.bunny", &iBunnyTable);
  assert(bf_status == BF_SUCCESS);

  bf_status = bfrtInfo->bfrtTableFromNameGet("SwitchEgress.bunny_e", &eBunnyTable);
  assert(bf_status == BF_SUCCESS);

  std::cout<<"got table ids"<<std::endl;

  // Get action Ids for route and nat actions
        //  bf_status = ipRouteTable->actionIdGet("SwitchIngress.route",
        //                                        &ipRoute_route_action_id);
        //  assert(bf_status == BF_SUCCESS);
        //
        //  bf_status =
        //      ipRouteTable->actionIdGet("SwitchIngress.nat", &ipRoute_nat_action_id);
        //  assert(bf_status == BF_SUCCESS);
    
  bf_status = iBunnyTable->actionIdGet("SwitchIngress.set_target", &ibunny_set_target_id);
  assert(bf_status == BF_SUCCESS);

  bf_status = eBunnyTable->actionIdGet("SwitchEgress.set_target_e", &ebunny_set_target_id);
  assert(bf_status == BF_SUCCESS);

  std::cout<<"got action ids"<<std::endl;

  // Get field-ids for key field and data fields
              //ig_md.robot_id: exact;
            //ig_md.actual_bunny: exact;
            //hdr.cur.jointId: exact;

            //bf_status = ipRouteTable->keyFieldIdGet("hdr.ipv4.dst_addr",
            //                                        &ipRoute_ip_dst_field_id);
            //assert(bf_status == BF_SUCCESS);
            //
            //bf_status = ipRouteTable->keyFieldIdGet("vrf", &ipRoute_vrf_field_id);
            //assert(bf_status == BF_SUCCESS);

    bf_status = iBunnyTable->keyFieldIdGet("ig_md.robot_id", &i_robot_id_field);
    assert(bf_status == BF_SUCCESS);

    bf_status = iBunnyTable->keyFieldIdGet("ig_md.actual_bunny", &i_actual_bunny_field);
    assert(bf_status == BF_SUCCESS);

    bf_status = iBunnyTable->keyFieldIdGet("hdr.cur.jointId", &i_joint_id_field);
    assert(bf_status == BF_SUCCESS);

    bf_status = eBunnyTable->keyFieldIdGet("eg_md.robot_id", &e_robot_id_field);
    assert(bf_status == BF_SUCCESS);

    bf_status = eBunnyTable->keyFieldIdGet("hdr.bridge.actual_bunny_id", &e_actual_bunny_field);
    assert(bf_status == BF_SUCCESS);

    bf_status = eBunnyTable->keyFieldIdGet("hdr.cur.jointId", &e_joint_id_field);
    assert(bf_status == BF_SUCCESS);

    std::cout<<"got key field ids"<<std::endl;

  /***********************************************************************
   * DATA FIELD ID GET FOR "route" ACTION
   **********************************************************************/
        //  bf_status =
        //      ipRouteTable->dataFieldIdGet("srcMac",
        //                                   ipRoute_route_action_id,
        //                                   &ipRoute_route_action_src_mac_field_id);
        //  assert(bf_status == BF_SUCCESS);
        //
        //  bf_status =
        //      ipRouteTable->dataFieldIdGet("dstMac",
        //                                   ipRoute_route_action_id,
        //                                   &ipRoute_route_action_dst_mac_field_id);
        //  assert(bf_status == BF_SUCCESS);
        //
        //  bf_status = ipRouteTable->dataFieldIdGet(
        //      "dst_port", ipRoute_route_action_id, &ipRoute_route_action_port_field_id);
        //  assert(bf_status == BF_SUCCESS);

  bf_status = iBunnyTable->dataFieldIdGet("next_id",
                                   ibunny_set_target_id,
                                   &iBunnyTable_next_id);
  assert(bf_status == BF_SUCCESS);

  bf_status = iBunnyTable->dataFieldIdGet("duration",
                                   ibunny_set_target_id,
                                   &iBunnyTable_duration);
  assert(bf_status == BF_SUCCESS);

  bf_status = eBunnyTable->dataFieldIdGet("tpos",
                                   ebunny_set_target_id,
                                   &eBunnyTable_tpos);
  assert(bf_status == BF_SUCCESS);

  bf_status = eBunnyTable->dataFieldIdGet("tspeed",
                                   ebunny_set_target_id,
                                   &eBunnyTable_tspeed);
  assert(bf_status == BF_SUCCESS);

  std::cout<<"got data field ids"<<std::endl;

  /***********************************************************************
   * DATA FIELD ID GET FOR "nat" ACTION
   **********************************************************************/
        //bf_status = ipRouteTable->dataFieldIdGet(
        //    "srcAddr", ipRoute_nat_action_id, &ipRoute_nat_action_ip_src_field_id);
        //assert(bf_status == BF_SUCCESS);
        //
        //bf_status = ipRouteTable->dataFieldIdGet(
        //    "dstAddr", ipRoute_nat_action_id, &ipRoute_nat_action_ip_dst_field_id);
        //assert(bf_status == BF_SUCCESS);
        //
        //bf_status = ipRouteTable->dataFieldIdGet(
        //    "dst_port", ipRoute_nat_action_id, &ipRoute_nat_action_port_field_id);
        //assert(bf_status == BF_SUCCESS);

  // Allocate key and data once, and use reset across different uses
        //  bf_status = ipRouteTable->keyAllocate(&bfrtTableKey);
        //  assert(bf_status == BF_SUCCESS);
        //
        //  bf_status = ipRouteTable->dataAllocate(&bfrtTableData);
        //  assert(bf_status == BF_SUCCESS);

  bf_status = iBunnyTable->keyAllocate(&iTableKey);
  assert(bf_status == BF_SUCCESS);

  bf_status = iBunnyTable->dataAllocate(&iTableData);
  assert(bf_status == BF_SUCCESS);

  bf_status = eBunnyTable->keyAllocate(&eTableKey);
  assert(bf_status == BF_SUCCESS);

  bf_status = eBunnyTable->dataAllocate(&eTableData);
  assert(bf_status == BF_SUCCESS);

  std::cout<<"helper variables allocated"<<std::endl;

}

/*******************************************************************************
 * Utility functions associated with "ipRoute" table in the P4 program.
 ******************************************************************************/

// This function sets the passed in ip_dst and vrf value into the key object
// passed using the setValue methods on the key object
void ipRoute_key_setup(const IpRouteKey &ipRoute_key,
                       bfrt::BfRtTableKey *table_key) {
  // Set value into the key object. Key type is "EXACT"
  auto bf_status = table_key->setValue(
      ipRoute_ip_dst_field_id, static_cast<uint64_t>(ipRoute_key.ipDstAddr));
  assert(bf_status == BF_SUCCESS);

  bf_status = table_key->setValue(ipRoute_vrf_field_id,
                                  static_cast<uint64_t>(ipRoute_key.vrf));
  assert(bf_status == BF_SUCCESS);

  return;
}

// This function sets the passed in "route" action data  into the
// data object associated with the ipRoute table
void ipRoute_data_setup_for_route(const IpRoute_routeData &ipRoute_data,
                                  bfrt::BfRtTableData *table_data) {
  // Set value into the data object
  auto bf_status = table_data->setValue(ipRoute_route_action_src_mac_field_id,
                                        ipRoute_data.srcMac);
  assert(bf_status == BF_SUCCESS);

  bf_status = table_data->setValue(ipRoute_route_action_dst_mac_field_id,
                                   ipRoute_data.dstMac);
  assert(bf_status == BF_SUCCESS);

  bf_status =
      table_data->setValue(ipRoute_route_action_port_field_id,
                           static_cast<uint64_t>(ipRoute_data.dst_port));
  assert(bf_status == BF_SUCCESS);

  return;
}

// This functiona sets the passed in "nat" acton data into the
// data object associated with the ipRoute table and "nat" action within the
// ipRoute table
void ipRoute_data_setup_for_nat(const IpRoute_natData &ipRoute_data,
                                bfrt::BfRtTableData *table_data) {
  // Set value into the data object
  auto bf_status =
      table_data->setValue(ipRoute_nat_action_ip_src_field_id,
                           static_cast<uint64_t>(ipRoute_data.srcAddr));
  assert(bf_status == BF_SUCCESS);

  bf_status = table_data->setValue(ipRoute_nat_action_ip_dst_field_id,
                                   static_cast<uint64_t>(ipRoute_data.dstAddr));
  assert(bf_status == BF_SUCCESS);

  bf_status =
      table_data->setValue(ipRoute_nat_action_port_field_id,
                           static_cast<uint64_t>(ipRoute_data.dst_port));
  assert(bf_status == BF_SUCCESS);

  return;
}


// ingress

void iBunny_key_setup(const bunny_key_t &key,
                       bfrt::BfRtTableKey *table_key) {
  // Set value into the key object. Key type is "EXACT"
  auto bf_status = table_key->setValue(
      i_robot_id_field, 
      static_cast<robot_id_t>(key.robot_id));
  assert(bf_status == BF_SUCCESS);

  bf_status = table_key->setValue(
      i_joint_id_field, 
      static_cast<joint_id_t>(key.jointId));
  assert(bf_status == BF_SUCCESS);

  bf_status = table_key->setValue(
      i_actual_bunny_field, 
      static_cast<bunny_id_t>(key.actual_bunny));
  assert(bf_status == BF_SUCCESS);

  return;
}

void iBunny_data_setup(const bunny_data_t &data,
                                  bfrt::BfRtTableData *table_data) {
  // Set value into the data object
  auto bf_status = table_data->setValue(
      iBunnyTable_next_id,
      static_cast<uint64_t>(data.next_id));
  assert(bf_status == BF_SUCCESS);

  bf_status = table_data->setValue(
      iBunnyTable_duration,
      static_cast<uint64_t>(data.duration));
  assert(bf_status == BF_SUCCESS);
  return;
}

void iBunny_entry_add(const bunny_key_t &key,
                                         const bunny_data_t &data,
                                         const bool &add) {
  // Adding a match entry with below mac Addr to be forwarded to the below port
  // Reset key and data before use
  iBunnyTable->keyReset(iTableKey.get());
  iBunnyTable->dataReset(ibunny_set_target_id, iTableData.get());

  // Fill in the Key and Data object
  iBunny_key_setup(key, iTableKey.get());
  iBunny_data_setup(data, iTableData.get());

  // Call table entry add API, if the request is for an add, else call modify
  bf_status_t status = BF_SUCCESS;
  if (add) {
    status = iBunnyTable->tableEntryAdd(
        *session, dev_tgt, *iTableKey, *iTableData);
  } else {
    status = iBunnyTable->tableEntryMod(
        *session, dev_tgt, *iTableKey, *iTableData);
  }
  assert(status == BF_SUCCESS);
  //session->sessionCompleteOperations();
}

void iBunny_entry_delete(const bunny_key_t &key) {
  // Reset key before use
  iBunnyTable->keyReset(iTableKey.get());

  iBunny_key_setup(key, iTableKey.get());

  auto status = iBunnyTable->tableEntryDel(*session, dev_tgt, *iTableKey);
  assert(status == BF_SUCCESS);
  //session->sessionCompleteOperations();
  return;
}

// egress

void eBunny_key_setup(const bunny_key_t &key,
                       bfrt::BfRtTableKey *table_key) {
  // Set value into the key object. Key type is "EXACT"
  auto bf_status = table_key->setValue(
      e_robot_id_field, 
      static_cast<robot_id_t>(key.robot_id));
  assert(bf_status == BF_SUCCESS);

  bf_status = table_key->setValue(
      e_joint_id_field, 
      static_cast<joint_id_t>(key.jointId));
  assert(bf_status == BF_SUCCESS);

  bf_status = table_key->setValue(
      e_actual_bunny_field, 
      static_cast<bunny_id_t>(key.actual_bunny));
  assert(bf_status == BF_SUCCESS);

  return;
}

void eBunny_data_setup(const bunny_target_t &data,
                                  bfrt::BfRtTableData *table_data) {
  // Set value into the data object
  auto bf_status = table_data->setValue(
      eBunnyTable_tpos,
      static_cast<uint64_t>(data.tpos));
  assert(bf_status == BF_SUCCESS);

  bf_status = table_data->setValue(
      eBunnyTable_tspeed,
      static_cast<uint64_t>(data.tspeed));
  assert(bf_status == BF_SUCCESS);
  return;
}

void eBunny_entry_add(const bunny_key_t &key,
                                         const bunny_target_t &data,
                                         const bool &add) {
  // Adding a match entry with below mac Addr to be forwarded to the below port
  // Reset key and data before use
  eBunnyTable->keyReset(eTableKey.get());
  eBunnyTable->dataReset(ebunny_set_target_id, eTableData.get());

  // Fill in the Key and Data object
  eBunny_key_setup(key, eTableKey.get());
  eBunny_data_setup(data, eTableData.get());

  // Call table entry add API, if the request is for an add, else call modify
  bf_status_t status = BF_SUCCESS;
  if (add) {
    status = eBunnyTable->tableEntryAdd(
        *session, dev_tgt, *eTableKey, *eTableData);
  } else {
    status = eBunnyTable->tableEntryMod(
        *session, dev_tgt, *eTableKey, *eTableData);
  }
  assert(status == BF_SUCCESS);
  //session->sessionCompleteOperations();
}

void eBunny_entry_delete(const bunny_key_t &key) {
  // Reset key before use
  eBunnyTable->keyReset(eTableKey.get());

  eBunny_key_setup(key, eTableKey.get());

  auto status = eBunnyTable->tableEntryDel(*session, dev_tgt, *eTableKey);
  assert(status == BF_SUCCESS);
  //session->sessionCompleteOperations();
  return;
}

void run_test_v2(){
    // insert 100 000 entry
    int k = 0;
    for (k=0;k<100000;k++){
        bunny_key_t key;
        bunny_data_t data;
        bunny_target_t target;
        bool add = true;

        key.robot_id = 0;
        key.jointId = k/65536;
        key.actual_bunny = k%65536;

        iBunny_entry_add(key,data,add);
        eBunny_entry_add(key,target,add);
    }

    // time insert and remove
    int i=0;
    for (int _x=0;_x<20;_x++){
        // start timer
        timeval start_time, end_time;
        gettimeofday(&start_time, NULL);
        auto status = session->beginBatch();
        assert(status==BF_SUCCESS);

        for (int _y=0;_y<10000;_y++){
            // remove based on i
            bunny_key_t key0;

            key0.robot_id = 0;
            key0.jointId = i/65536;
            key0.actual_bunny = i%65536;

            iBunny_entry_delete(key0);
            eBunny_entry_delete(key0);

            i++;

            // add based on k  
            bunny_key_t key;
            bunny_data_t data;
            bunny_target_t target;
            bool add = true;

            key.robot_id = 0;
            key.jointId = k/65536;
            key.actual_bunny = k%65536;

            iBunny_entry_add(key,data,add);
            eBunny_entry_add(key,target,add);

            k++;

        }   

        // stop timer
        status = session->endBatch(true);
        assert(status==BF_SUCCESS);
        session->sessionCompleteOperations();

        gettimeofday(&end_time, NULL);
        auto diff = ((end_time.tv_sec * 1000000 + end_time.tv_usec) - (start_time.tv_sec * 1000000 + start_time.tv_usec));
        std::cout<<"removeadd "<<10000<< " "<<_x<<" "<<diff<<std::endl;

    }

}

void run_tests(const int record_count,const int repeat_count){
    //std::cout << "### insert " << record_count << " records" <<std::endl;
    
    for (int i=0;i<repeat_count;i++){
        //std::cout<<"test "<<i << std::endl;
        timeval start_time, end_time;
        //auto start_time = std::chrono::steady_clock::now();
        gettimeofday(&start_time, NULL);

        auto status = session->beginBatch();
        assert(status==BF_SUCCESS);
        for (int k=0;k<=record_count;k++){
            bunny_key_t key;
            bunny_data_t data;
            bunny_target_t target;
            bool add = true;

            key.robot_id = i;
            key.jointId = k/65536;
            key.actual_bunny = k%65536;

            //std::cout <<"add " << i << " "<<k<<std::endl;
            
            iBunny_entry_add(key,data,add);
            eBunny_entry_add(key,target,add);
        }

        status = session->endBatch(true);
        assert(status==BF_SUCCESS);
        session->sessionCompleteOperations();

        //auto end_time = std::chrono::steady_clock::now();
        gettimeofday(&end_time, NULL);
        auto diff = ((end_time.tv_sec * 1000000 + end_time.tv_usec) - (start_time.tv_sec * 1000000 + start_time.tv_usec));
        //std::chrono::duration<double> diff = end_time-start_time;
        std::cout<<"add "<<record_count<< " "<<i<<" "<<diff<<std::endl;
        //std::cout << diff.count() << std::endl;

    }

    session->sessionCompleteOperations();


    //std::cout << "### remove  " << record_count << "  records" <<std::endl;
    
    for (int i=0;i<repeat_count;i++){
        //std::cout<<"test "<< i << std::endl;

        timeval start_time, end_time;
        //auto start_time = std::chrono::steady_clock::now();
        gettimeofday(&start_time, NULL);

        auto status = session->beginBatch();
        assert(status==BF_SUCCESS);

        for (int k=0;k<=record_count;k++){
            bunny_key_t key;

            key.robot_id = i;
            key.jointId = k/65536;
            key.actual_bunny = k%65536;

            //std::cout <<"remove " << i << " "<<k<<std::endl;

            iBunny_entry_delete(key);
            eBunny_entry_delete(key);
        }

        status = session->endBatch(true);
        assert(status==BF_SUCCESS);
        session->sessionCompleteOperations();

        //auto end_time = std::chrono::steady_clock::now();
        gettimeofday(&end_time, NULL);
        auto diff = ((end_time.tv_sec * 1000000 + end_time.tv_usec) - (start_time.tv_sec * 1000000 + start_time.tv_usec));
        //std::chrono::duration<double> diff = end_time-start_time;
        std::cout<<"remove "<<record_count<< " "<<i<<" "<<diff<<std::endl;
        //std::cout << diff.count() << std::endl;

    }

    session->sessionCompleteOperations();


}







// This function adds or modifies an entry in the ipRoute table with "route"
// action. The workflow is similar for either table entry add or modify
void ipRoute_entry_add_modify_with_route(const IpRouteKey &ipRoute_key,
                                         const IpRoute_routeData &ipRoute_data,
                                         const bool &add) {
  // Adding a match entry with below mac Addr to be forwarded to the below port
  // Reset key and data before use
  ipRouteTable->keyReset(bfrtTableKey.get());
  ipRouteTable->dataReset(ipRoute_route_action_id, bfrtTableData.get());

  // Fill in the Key and Data object
  ipRoute_key_setup(ipRoute_key, bfrtTableKey.get());
  ipRoute_data_setup_for_route(ipRoute_data, bfrtTableData.get());

  // Call table entry add API, if the request is for an add, else call modify
  bf_status_t status = BF_SUCCESS;
  if (add) {
    status = ipRouteTable->tableEntryAdd(
        *session, dev_tgt, *bfrtTableKey, *bfrtTableData);
  } else {
    status = ipRouteTable->tableEntryMod(
        *session, dev_tgt, *bfrtTableKey, *bfrtTableData);
  }
  assert(status == BF_SUCCESS);
  session->sessionCompleteOperations();
}

// This function adds or modifies an entry in the ipRoute table with "nat"
// action. The workflow is similar for either table entry add or modify
void ipRoute_entry_add_modify_with_nat(const IpRouteKey &ipRoute_key,
                                       const IpRoute_natData &ipRoute_data,
                                       const bool &add) {
  // Reset key and data before use
  ipRouteTable->keyReset(bfrtTableKey.get());
  ipRouteTable->dataReset(ipRoute_nat_action_id, bfrtTableData.get());

  ipRoute_key_setup(ipRoute_key, bfrtTableKey.get());
  ipRoute_data_setup_for_nat(ipRoute_data, bfrtTableData.get());

  // Call table entry add API, if the request is for an add, else call modify
  bf_status_t status = BF_SUCCESS;
  if (add) {
    status = ipRouteTable->tableEntryAdd(
        *session, dev_tgt, *bfrtTableKey, *bfrtTableData);
  } else {
    status = ipRouteTable->tableEntryMod(
        *session, dev_tgt, *bfrtTableKey, *bfrtTableData);
  }
  assert(status == BF_SUCCESS);
  session->sessionCompleteOperations();
  return;
}

// This function process the entry obtained by a get call for a "route" action
// and populates the IpRoute_routeData structure
void ipRoute_process_route_entry_get(const bfrt::BfRtTableData &data,
                                     IpRoute_routeData *route_data) {
  bf_status_t status = BF_SUCCESS;

  status =
      data.getValue(ipRoute_route_action_src_mac_field_id, &route_data->srcMac);
  assert(status == BF_SUCCESS);

  status =
      data.getValue(ipRoute_route_action_dst_mac_field_id, &route_data->dstMac);
  assert(status == BF_SUCCESS);

  uint64_t port;
  status = data.getValue(ipRoute_route_action_port_field_id, &port);
  route_data->dst_port = static_cast<uint16_t>(port);
  assert(status == BF_SUCCESS);

  return;
}

// This function process the entry obtained by a get call for a "nat" action
// and populates the IpRoute_natData structure
void ipRoute_process_nat_entry_get(const bfrt::BfRtTableData &data,
                                   IpRoute_natData *nat_data) {
  bf_status_t status = BF_SUCCESS;

  uint64_t srcAddr;
  status = data.getValue(ipRoute_nat_action_ip_src_field_id, &srcAddr);
  assert(status == BF_SUCCESS);
  nat_data->srcAddr = static_cast<uint32_t>(srcAddr);

  uint64_t dstAddr;
  status = data.getValue(ipRoute_nat_action_ip_dst_field_id, &dstAddr);
  assert(status == BF_SUCCESS);
  nat_data->dstAddr = static_cast<uint32_t>(dstAddr);

  uint64_t dst_port;
  status = data.getValue(ipRoute_nat_action_port_field_id, &dst_port);
  assert(status == BF_SUCCESS);
  nat_data->dst_port = static_cast<uint16_t>(dst_port);

  return;
}

// This function processes the entry obtained by a get call. Based on the action
// id the data object is intepreted.
void ipRoute_process_entry_get(const bfrt::BfRtTableData &data,
                               IpRouteData *ipRoute_data) {
  // First get actionId, then based on that, fill in appropriate fields
  bf_status_t bf_status;
  bf_rt_id_t action_id;

  bf_status = data.actionIdGet(&action_id);
  assert(bf_status == BF_SUCCESS);

  if (action_id == ipRoute_route_action_id) {
    ipRoute_process_route_entry_get(data, &ipRoute_data->data.route_data);
  } else if (action_id == ipRoute_nat_action_id) {
    ipRoute_process_nat_entry_get(data, &ipRoute_data->data.nat_data);
  }
  return;
}

// This function reads an entry specified by the ipRoute_key, and fills in the
// passedin IpRoute object
void ipRoute_entry_get(const IpRouteKey &ipRoute_key, IpRouteData *data) {
  // Reset key and data before use
  ipRouteTable->keyReset(bfrtTableKey.get());
  // Data reset is done without action-id, since the action-id is filled in by
  // the get function
  ipRouteTable->dataReset(bfrtTableData.get());

  ipRoute_key_setup(ipRoute_key, bfrtTableKey.get());

  bf_status_t status = BF_SUCCESS;
  // Entry get from hardware with the flag set to read from hardware
  auto flag = bfrt::BfRtTable::BfRtTableGetFlag::GET_FROM_HW;
  status = ipRouteTable->tableEntryGet(
      *session, dev_tgt, *bfrtTableKey, flag, bfrtTableData.get());
  assert(status == BF_SUCCESS);
  session->sessionCompleteOperations();

  ipRoute_process_entry_get(*bfrtTableData, data);

  return;
}

// This function deletes an entry specified by the ipRoute_key
void ipRoute_entry_delete(const IpRouteKey &ipRoute_key) {
  // Reset key before use
  ipRouteTable->keyReset(bfrtTableKey.get());

  ipRoute_key_setup(ipRoute_key, bfrtTableKey.get());

  auto status = ipRouteTable->tableEntryDel(*session, dev_tgt, *bfrtTableKey);
  assert(status == BF_SUCCESS);
  session->sessionCompleteOperations();
  return;
}

// Function to iterate over all the entries in the table
void table_iterate() {
  // Table iteration involves the following
  //    1. Use the getFirst API to get the first entry
  //    2. Use the tableUsageGet API to get the number of entries currently in
  //    the table.
  //    3. Use the number of entries returned in step 2 and pass it as a
  //    parameter to getNext_n (as n) to get all the remaining entries
  std::unique_ptr<BfRtTableKey> first_key;
  std::unique_ptr<BfRtTableData> first_data;

  auto bf_status = ipRouteTable->keyAllocate(&first_key);
  assert(bf_status == BF_SUCCESS);

  bf_status = ipRouteTable->dataAllocate(&first_data);
  assert(bf_status == BF_SUCCESS);

  auto flag = bfrt::BfRtTable::BfRtTableGetFlag::GET_FROM_HW;

  bf_status = ipRouteTable->tableEntryGetFirst(
      *session, dev_tgt, flag, first_key.get(), first_data.get());
  assert(bf_status == BF_SUCCESS);
  session->sessionCompleteOperations();

  // Process the first entry
  IpRouteData route_data;
  ipRoute_process_entry_get(*first_data, &route_data);

  // Get the usage of table
  uint32_t entry_count = 0;
  bf_status =
      ipRouteTable->tableUsageGet(*session, dev_tgt, flag, &entry_count);
  assert(bf_status == BF_SUCCESS);

  if (entry_count == 1) {
    return;
  }

  BfRtTable::keyDataPairs key_data_pairs;
  std::vector<std::unique_ptr<BfRtTableKey>> keys(entry_count - 1);
  std::vector<std::unique_ptr<BfRtTableData>> data(entry_count - 1);

  for (unsigned i = 0; i < entry_count - 1; ++i) {
    bf_status = ipRouteTable->keyAllocate(&keys[i]);
    assert(bf_status == BF_SUCCESS);

    bf_status = ipRouteTable->dataAllocate(&data[i]);
    assert(bf_status == BF_SUCCESS);

    key_data_pairs.push_back(std::make_pair(keys[i].get(), data[i].get()));
  }

  // Get next N
  uint32_t num_returned = 0;
  bf_status = ipRouteTable->tableEntryGetNext_n(*session,
                                                dev_tgt,
                                                *first_key.get(),
                                                entry_count - 1,
                                                flag,
                                                &key_data_pairs,
                                                &num_returned);
  assert(bf_status == BF_SUCCESS);
  assert(num_returned == entry_count - 1);
  session->sessionCompleteOperations();

  // Process the rest of the entries
  for (unsigned i = 0; i < entry_count - 1; ++i) {
    ipRoute_process_entry_get(*data[i], &route_data);
    // Do any required processing with the obtained data and key
  }
  return;
}


}  // tna_exact_match
}  // examples
}  // bfrt

static void parse_options(bf_switchd_context_t *switchd_ctx,
                          int argc,
                          char **argv) {
  int option_index = 0;
  enum opts {
    OPT_INSTALLDIR = 1,
    OPT_CONFFILE,
  };
  static struct option options[] = {
      {"help", no_argument, 0, 'h'},
      {"install-dir", required_argument, 0, OPT_INSTALLDIR},
      {"conf-file", required_argument, 0, OPT_CONFFILE}};

  while (1) {
    int c = getopt_long(argc, argv, "h", options, &option_index);

    if (c == -1) {
      break;
    }
    switch (c) {
      case OPT_INSTALLDIR:
        switchd_ctx->install_dir = strdup(optarg);
        printf("Install Dir: %s\n", switchd_ctx->install_dir);
        break;
      case OPT_CONFFILE:
        switchd_ctx->conf_file = strdup(optarg);
        printf("Conf-file : %s\n", switchd_ctx->conf_file);
        break;
      case 'h':
      case '?':
        printf("tna_exact_match \n");
        printf(
            "Usage : tna_exact_match --install-dir <path to where the SDE is "
            "installed> --conf-file <full path to the conf file "
            "(tna_exact_match.conf)\n");
        exit(c == 'h' ? 0 : 1);
        break;
      default:
        printf("Invalid option\n");
        exit(0);
        break;
    }
  }
  if (switchd_ctx->install_dir == NULL) {
    printf("ERROR : --install-dir must be specified\n");
    exit(0);
  }

  if (switchd_ctx->conf_file == NULL) {
    printf("ERROR : --conf-file must be specified\n");
    exit(0);
  }
}



int main(int argc, char **argv) {
  bf_switchd_context_t *switchd_ctx;
  if ((switchd_ctx = (bf_switchd_context_t *)calloc(
           1, sizeof(bf_switchd_context_t))) == NULL) {
    printf("Cannot Allocate switchd context\n");
    exit(1);
  }
  parse_options(switchd_ctx, argc, argv);
  switchd_ctx->running_in_background = true;
  bf_status_t status = bf_switchd_lib_init(switchd_ctx);

  // Do initial set up
  std::cout<<"################################################## SET UP STARTED"<<std::endl;
  bfrt::examples::tna_exact_match::setUp();
  std::cout<<"################################################## SET UP FINISHED"<<std::endl;
  // Do table level set up
  std::cout<<"################################################## TABLE SET UP STARTED"<<std::endl;
  bfrt::examples::tna_exact_match::tableSetUp();
  std::cout<<"################################################## TABLE SET UP FINISHED"<<std::endl;
  
  std::cout<<"################################################## TESTS STARTED"<<std::endl;
  
  bfrt::examples::tna_exact_match::run_test_v2();

  for (int i=0;i<50;i++){
      bfrt::examples::tna_exact_match::run_tests(100,20);
      bfrt::examples::tna_exact_match::run_tests(1000,20);
  }
  for (int i=0;i<100;i++)
      bfrt::examples::tna_exact_match::run_tests(10000,10);
  for (int i=0;i<500;i++)  
      bfrt::examples::tna_exact_match::run_tests(100000,2);
  std::cout<<"################################################## TESTS FINISHED"<<std::endl;
  
  return status;
}

