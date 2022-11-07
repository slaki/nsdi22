#include <core.p4>
#if __TARGET_TOFINO__ == 2
#include <t2na.p4>
#else
#include <tna.p4>
#endif

//#define ROBOT_ID_LEN 8
//#define BUNNY_ID_LEN 16
//#define MAX_ROBOTS 100
//#define BUNNY_TABLE_SIZE 100000
//#define FUNCTION_SIZE 12000
//#define SPEED_LIMIT_SIZE 640

#include "params.p4"

#define TIME_T bit<32>
#define ROBOT_ID_T bit<ROBOT_ID_LEN>
#define BUNNY_ID_T bit<BUNNY_ID_LEN>



// HEADERS AND TYPES ************************************************************

header ethernet_t {
    bit<48>   dstAddr;
    bit<48>   srcAddr;
    bit<16>   etherType;
}

header ipv4_t {
    bit<4>  version;
    bit<4>  ihl;
    bit<8>  diffserv;
    bit<16> totalLen;
    bit<16> identification;
    bit<1> _reserved;
    bit<1> dont_fragment;
    bit<1> more_fragments;
    bit<13> fragOffset;
    bit<8>  ttl;
    bit<8>  protocol;
    bit<16> hdrChecksum;
    bit<32> srcAddr;
    bit<32> dstAddr;
}

struct udp_port_t{
    bit<8> first_half;
    bit<8> robot_id;
}

header udp_t{
    bit<16> srcPort;
    bit<16> dstPort;
    bit<16> len;
    bit<16> csum;
}

header ptp_t{
    bit<8>	messageType;
	bit<8>	versionPTP;
	bit<16>	len;
	bit<8>	domain;
	bit<8>	reserved;
	bit<16>	flags;
	bit<32>	correction_0;
	bit<32>	correction_1;
	bit<32>	reserved2;
	bit<8>	oui_0;
	bit<8>	oui_1;
	bit<8>	oui_2;
	bit<8>	uuid_0;
	bit<8>	uuid_1;
	bit<8>	uuid_2;
	bit<8>	uuid_3;
	bit<8>	uuid_4;
	bit<16>	ptpNodePort;
	bit<16>	sequenceId;
	bit<8>	ctrl;
	bit<8>	logMessageInterval;
}

header cur_t{
    bit<8> jointId;
    bit<64> position;
    bit<64> speed;
}

header resubmit_t{ 
    BUNNY_ID_T bunny_id;
    bit<48> _padding;
}

header bridge_t{
    BUNNY_ID_T actual_bunny_id;
}

header control_t{
    bit<8> command;
    bit<8> log;
    TIME_T actual;
    TIME_T end;
}

// #meta
struct ingress_metadata_t {
    BUNNY_ID_T actual_bunny;
    BUNNY_ID_T next_bunny;
    TIME_T bunny_duration;
    TIME_T end_time;
    TIME_T actual_time;
    bit<1> resubmit_needed;
    ROBOT_ID_T robot_id;
}

struct egress_metadata_t {
    bit<64> u_target_position;
    bit<64> u_target_speed;
    bit<64> u_actual_speed;
    bit<64> target_position;
    bit<64> target_speed;
    ROBOT_ID_T robot_id;
}


struct header_t {
    ethernet_t ethernet;
    ipv4_t ipv4;
    udp_t udp;
    ptp_t ptp;
    cur_t cur;
    
    resubmit_t resubmit;
    bridge_t bridge;

    control_t ctrl;
}

// INGRESS ************************************************************

parser TofinoIngressParser(
        packet_in pkt,
        out header_t hdr,
        out ingress_intrinsic_metadata_t ig_intr_md) {

    state start {
        pkt.extract(ig_intr_md);
        transition select(ig_intr_md.resubmit_flag) {
            1 : parse_resubmit;
            0 : parse_port_metadata;
        }
    }

    state parse_resubmit {
        // Parse resubmitted packet here.
        pkt.extract(hdr.resubmit);
        transition accept;
    }

    state parse_port_metadata {
#if __TARGET_TOFINO__ == 2
        pkt.advance(192);
#else
        pkt.advance(64);
#endif
        transition accept;
    }
}


parser TofinoEgressParser(
        packet_in pkt,
        out egress_intrinsic_metadata_t eg_intr_md) {

    state start {
        pkt.extract(eg_intr_md);
        transition accept;
    }

}


parser SwitchIngressParser(
        packet_in pkt,
        out header_t hdr,
        out ingress_metadata_t ig_md,
        out ingress_intrinsic_metadata_t ig_intr_md) {


    TofinoIngressParser() tofino_parser;
	
    state start {
	    tofino_parser.apply(pkt, hdr, ig_intr_md);
        transition parse_ethernet;
    }

    state parse_ethernet {
        pkt.extract(hdr.ethernet);
        transition select(hdr.ethernet.etherType) {
            16w0x800: parse_ipv4;
            16w0x1234: parse_control;
            default: accept;
        }
    }

    state parse_ipv4 {
        pkt.extract(hdr.ipv4);
        transition select(hdr.ipv4.protocol){
            //6: parse_tcp;
            17: parse_udp;
            default: accept; 
        } 
    }

    state parse_udp{
        pkt.extract(hdr.udp);
        transition select(hdr.udp.srcPort){
            319 : parse_ptp;
            default : decide_cur;
        }
    }

    state parse_ptp{
        pkt.extract(hdr.ptp);
        transition decide_cur;
    }

    state decide_cur{
        transition select(hdr.udp.srcPort){
//            50004 : parse_cur_0;
            #if ROBOT_ID_LEN == 16
                16w0xFFFF &&& 16w0xF000 : parse_cur;
            #else
                16w0xFFFF &&& 16w0xFF00 : parse_cur;
            #endif

            default: accept; 
        }  
    }

    state parse_cur_0{
        //ig_md.robot_id = 0;
        pkt.extract(hdr.cur);
        transition accept;
    }

    state parse_cur{
        //ig_md.robot_id = hdr.udp.srcPort[7:0];
        pkt.extract(hdr.cur);
        transition accept;
    }


    state parse_control{
        pkt.extract(hdr.ctrl);
        transition accept;
    }

}

control SwitchIngress(
        inout header_t hdr,
        inout ingress_metadata_t ig_md,
        in ingress_intrinsic_metadata_t ig_intr_md,
        in ingress_intrinsic_metadata_from_parser_t ig_prsr_md,
        inout ingress_intrinsic_metadata_for_deparser_t ig_dprsr_md,
        inout ingress_intrinsic_metadata_for_tm_t ig_tm_md) {

    // ***** REGISTERS ******

    Register<BUNNY_ID_T,ROBOT_ID_T>(MAX_ROBOTS) r_actual_bunny;
    RegisterAction<BUNNY_ID_T,_,BUNNY_ID_T>(r_actual_bunny) get_actual_bunny = {
        void apply(inout BUNNY_ID_T data,out BUNNY_ID_T new_value){
           new_value = data;
        }
    };
    RegisterAction<BUNNY_ID_T,_,BUNNY_ID_T>(r_actual_bunny) update_actual_bunny = {
        void apply(inout BUNNY_ID_T data,out BUNNY_ID_T new_value){
           data = ig_md.actual_bunny;
           new_value = data;
        }
    };


    Register<BUNNY_ID_T,ROBOT_ID_T>(MAX_ROBOTS) r_next_bunny;
    RegisterAction<BUNNY_ID_T,_,BUNNY_ID_T>(r_next_bunny) get_next_bunny = {
        void apply(inout BUNNY_ID_T data,out BUNNY_ID_T new_value){
           new_value = data;
        }
    };
    RegisterAction<BUNNY_ID_T,_,BUNNY_ID_T>(r_next_bunny) update_next_bunny = {
        void apply(inout BUNNY_ID_T data,out BUNNY_ID_T new_value){
           data = ig_md.next_bunny;
           new_value = data;
        }
    };


    Register<TIME_T,ROBOT_ID_T>(MAX_ROBOTS) end_time;
    RegisterAction<TIME_T,_,TIME_T>(end_time) get_end_time = {
        void apply(inout TIME_T data,out TIME_T new_value){
           if (ig_intr_md.resubmit_flag==1){
               data = data + ig_md.bunny_duration;
           }
           new_value = data;
        }
    };
    RegisterAction<TIME_T,_,TIME_T>(end_time) update_end_time = {
        void apply(inout TIME_T data,out TIME_T new_value){
           data = data + ig_md.bunny_duration;
           new_value = data;
        }
    };
    RegisterAction<TIME_T,_,TIME_T>(end_time) init_end_time = {
        void apply(inout TIME_T data,out TIME_T new_value){
           data = ig_md.actual_time;
           new_value = data;
        }
    };
    // ***** ACTIONS AND TABLES *****

    action send_back(){
        /* Swap the MAC addresses */
        bit<48> tmp = hdr.ethernet.dstAddr;
        hdr.ethernet.dstAddr = hdr.ethernet.srcAddr;
        hdr.ethernet.srcAddr = tmp;
        //hdr.ethernet.etherType = (bit<16>)ig_intr_md.ingress_port;
        
        /* Send the packet back to the port it came from */
        ig_tm_md.ucast_egress_port = ig_intr_md.ingress_port;
    }

    action set_target(BUNNY_ID_T next_id,
                      TIME_T duration){
        
        ig_md.bunny_duration = duration;
        ig_md.next_bunny = next_id;
    }

    action log(){}

    table bunny{
        actions = {
            set_target;
            log;
        }
        key = {
            ig_md.robot_id: exact;
            ig_md.actual_bunny: exact;
            hdr.cur.jointId: exact;
        }
        size = BUNNY_TABLE_SIZE;
        const default_action = log;
        //const entries = {
            //{0,0,0} : set_target(1,152590);
            //{0,1,0} : set_target(0,152590);
       // }
    }


    action init_actual_bunny_from_register(){
        ig_md.actual_bunny = get_actual_bunny.execute(ig_md.robot_id);
    }

    @stage(6)
    table call_init_actual_bunny_from_register{
        actions = { init_actual_bunny_from_register;}
        key = {} 
        const default_action = init_actual_bunny_from_register;
    }

    action init_next_bunny_from_register(){
        ig_md.next_bunny = get_next_bunny.execute(ig_md.robot_id);
    }

    @stage(5)
    table call_init_next_bunny_from_register{
        actions = { init_next_bunny_from_register;}
        key = {} 
        const default_action = init_next_bunny_from_register;
    }

    action change_next_bunny(BUNNY_ID_T bunny_id){
        ig_md.next_bunny = bunny_id;
    }

    table railway_switch{
        actions = {
            change_next_bunny; 
            NoAction;
        }
        key = {
            ig_md.robot_id: exact;
            ig_md.actual_bunny: exact;
        }
        size = 1024;
        const default_action = NoAction;
    }
    
    action init_end_time_from_register(){
        ig_md.end_time = get_end_time.execute(ig_md.robot_id);
    }

    @stage(3)
    table call_init_end_time_from_register{
        actions = { init_end_time_from_register;}
        key = {} 
        const default_action = init_end_time_from_register;
    }

    action time_check_calculations__1(){
        ig_md.end_time = ig_md.end_time + 0x00ffffff;
        //ig_md.end_time = ig_md.end_time + 0x00ffffffffffffff;
    }

    action time_check_calculations__2(){
        ig_md.end_time = ig_md.end_time - ig_md.actual_time;
    }

    action set_resubmit(bit<1> needed){
        ig_md.resubmit_needed = needed;
        // NOTE: if we ask a resubmit this meta does not matter otherwise it is the correct value
        //ig_md.actual_bunny = get_actual_bunny.execute(ig_md.robot_id);
    }

    @stage(4)
    table resubmit_setter{
        actions = {
            set_resubmit;
        }
        key = { 
            ig_intr_md.resubmit_flag: exact;
            ig_md.end_time: ternary;
        }
        const default_action = set_resubmit(0);
        const entries = {
            (1, _ ) : set_resubmit(0);
            (0, 0x00000000&&&0xff000000) : set_resubmit(1);
//            (0, 0x00000000&&&0xff00000000000000) : set_resubmit(1);
        }
    }

    action init_actual_bunny_from_resubmit(){
        ig_md.actual_bunny = hdr.resubmit.bunny_id;
    }

    action update_actual_bunny_in_register(){
        update_actual_bunny.execute(ig_md.robot_id);
    }

    @stage(6)
    table call_update_actual_bunny_in_register{
        actions = { update_actual_bunny_in_register;}
        key = {} 
        const default_action = update_actual_bunny_in_register;
    }

    action update_next_bunny_in_register(){
        update_next_bunny.execute(ig_md.robot_id);
    }

    @stage(5)
    table call_update_next_bunny_in_register{
        actions = { update_next_bunny_in_register;}
        key = {} 
        const default_action = update_next_bunny_in_register;
    }

    action update_end_time_in_register(){
        ig_md.end_time = update_end_time.execute(ig_md.robot_id);
    }

    @stage(3)
    table call_update_end_time_in_register{
        actions = { update_end_time_in_register; }
        key = {} 
        const default_action = update_end_time_in_register;
    }

    @stage(3)
    table call_update_end_time_in_register_v2{
        actions = { update_end_time_in_register; }
        key = {} 
        const default_action = update_end_time_in_register;
    }

    action send(PortId_t port){
        ig_tm_md.ucast_egress_port = port;
    }

    action init_actual_time(){
        ig_md.actual_time = (TIME_T) ig_prsr_md.global_tstamp[47:16];
    }

    action swap_ipv4_addresses(){
        bit<32> tmp = hdr.ipv4.srcAddr;
        hdr.ipv4.srcAddr = hdr.ipv4.dstAddr;
        hdr.ipv4.dstAddr = tmp;
    }

    // #ingress
    apply {        
        if (hdr.ctrl.isValid()){
            ig_md.bunny_duration = (TIME_T) hdr.ctrl.end;
            hdr.ctrl.actual = (TIME_T) ig_prsr_md.global_tstamp[47:16];
            call_update_end_time_in_register_v2.apply();
            hdr.ctrl.end = ig_md.end_time;
            //set_resubmit(1);
            hdr.ctrl.log = 5;
            send_back();
        }
        else if (hdr.cur.isValid()){
            #if ROBOT_ID_LEN == 8
                ig_md.robot_id = hdr.udp.srcPort[7:0];
            #else
                ig_md.robot_id = hdr.udp.srcPort & 16w0x0FFF;
            #endif
            if (ig_intr_md.resubmit_flag==0){
                // ** it is a new packet
                init_actual_time();
                call_init_next_bunny_from_register.apply();
                call_init_end_time_from_register.apply();
                time_check_calculations__1();
                time_check_calculations__2();
                resubmit_setter.apply();
                call_init_actual_bunny_from_register.apply();
                railway_switch.apply();
            }
            else{
                // ** it is a resubmitted packet
                init_actual_bunny_from_resubmit();
                bunny.apply();
                call_update_next_bunny_in_register.apply();
                call_update_actual_bunny_in_register.apply();
                call_update_end_time_in_register.apply();
            }
            send_back();
        }
        else{ // simple forwarding
            if (hdr.ipv4.srcAddr==0x14000005){
                swap_ipv4_addresses();
                send_back();
            }
            else if (ig_intr_md.ingress_port == 48)
            {
                //send(52);
                send(48);
            }
	        else{
                //send(48);
                send(52);
            } 
        }
        hdr.bridge = {ig_md.actual_bunny};
        hdr.bridge.setValid();
    }
}

control SwitchIngressDeparser(
        packet_out pkt,
        inout header_t hdr,
        in ingress_metadata_t ig_md,
        in ingress_intrinsic_metadata_for_deparser_t ig_dprsr_md) {
    
    Resubmit() r;

    apply {
        if (ig_md.resubmit_needed==1)
            r.emit<resubmit_t>({ig_md.next_bunny,0});
        pkt.emit(hdr.bridge);
        pkt.emit(hdr.ethernet);
        pkt.emit(hdr.ipv4);
        pkt.emit(hdr.udp);
        pkt.emit(hdr.ptp);
        pkt.emit(hdr.cur);
        pkt.emit(hdr.ctrl);
    }
}

// EGRESS ************************************************************

parser SwitchEgressParser(
        packet_in pkt,
        out header_t hdr,
        out egress_metadata_t eg_md,
        out egress_intrinsic_metadata_t eg_intr_md) {

    TofinoEgressParser() tofino_parser;

    state start {
	    tofino_parser.apply(pkt, eg_intr_md);
        transition parse_bridge_data;
    }

    state parse_bridge_data{
        pkt.extract(hdr.bridge);
        transition parse_ethernet;
    }

    state parse_ethernet {
        pkt.extract(hdr.ethernet);
        transition select(hdr.ethernet.etherType) {
            16w0x800: parse_ipv4;
            default: accept;
        }
    }

    state parse_ipv4 {
        pkt.extract(hdr.ipv4);
        transition select(hdr.ipv4.protocol){
            //6: parse_tcp;
            17: parse_udp;
            default: accept; 
        } 
    }

    state parse_udp{
        pkt.extract(hdr.udp);
        transition select(hdr.udp.srcPort){
            319 : parse_ptp;
            default : decide_cur;
        }
    }

    state parse_ptp{
        pkt.extract(hdr.ptp);
        transition decide_cur;
    }

    state decide_cur{
        transition select(hdr.udp.srcPort){
//            50004 : parse_cur_0;
            #if ROBOT_ID_LEN == 16
                16w0xFFFF &&& 16w0xF000 : parse_cur;
            #else
                16w0xFFFF &&& 16w0xFF00 : parse_cur;
            #endif

            default: accept; 
        }  
    }

    state parse_cur{
        //eg_md.robot_id = hdr.udp.srcPort[7:0];
        pkt.extract(hdr.cur);
        transition accept;
    }


}

control SwitchEgress(
        inout header_t hdr,
        inout egress_metadata_t eg_md,
        in egress_intrinsic_metadata_t eg_intr_md,
        in egress_intrinsic_metadata_from_parser_t eg_intr_from_prsr,
        inout egress_intrinsic_metadata_for_deparser_t eg_intr_md_for_dprsr,
        inout egress_intrinsic_metadata_for_output_port_t eg_intr_md_for_oport) {

    action set_target_e(bit<64> tpos,bit<64> tspeed){
        eg_md.target_position = tpos;
        eg_md.target_speed =  tspeed;
    }

    action log_e(){}

    table bunny_e{
        actions = {
            set_target_e;
            log_e;
        }
        key = {
            eg_md.robot_id: exact;
            hdr.bridge.actual_bunny_id: exact;
            hdr.cur.jointId: exact;
        }
        size = BUNNY_TABLE_SIZE;
        const default_action = log_e;
        //const entries = {
            //{0,0,0} : set_target_e(0b0000000000000000000000000000000000000000101000101111100110000011,0);
            //{0,1,0} : set_target_e(0b1111111111111111111111111111111111111111010111010000011001111101,0);
        //}
    }

    action calculate_diff(){
        eg_md.target_position = eg_md.target_position - hdr.cur.position;
    }

    action set_new_speed(){
        hdr.cur.speed = eg_md.target_position + eg_md.target_position;
    }

    action double_speed(){
        hdr.cur.speed = hdr.cur.speed+hdr.cur.speed;
    }

    action clear_checksums(){
        hdr.ipv4.hdrChecksum = 0;
        hdr.udp.csum = 0;
    }

    action swap_ipv4_addresses(){
        bit<32> tmp = hdr.ipv4.srcAddr;
        hdr.ipv4.srcAddr = hdr.ipv4.dstAddr;
        hdr.ipv4.dstAddr = tmp;
    }

    action swap_udp_ports(){
        bit<16> tmp = hdr.udp.srcPort;
        hdr.udp.srcPort = hdr.udp.dstPort;
        hdr.udp.dstPort = tmp;
    }



    action override_speed(bit<64> speed){
        hdr.cur.speed = speed;
    }

    action allow_speed(){
        
    }

    table speed_limit{
        actions = { 
            allow_speed; 
            override_speed;
        }
        key = {
            hdr.cur.jointId : exact;
            hdr.cur.speed : ternary;
        }
        const default_action = allow_speed;
        size = SPEED_LIMIT_SIZE;
        const entries = {
            #include "a_speed_limit_entries.p4"
        }
    }

    action update_target_speed(bit<64> d){
        eg_md.target_speed = d;
    }

    table target_speed_function{
        actions = {
            update_target_speed;
            NoAction;
        }
        key = {
            eg_md.target_speed: ternary;
        }
        const default_action = NoAction;
        size = FUNCTION_SIZE;
    }

    action update_actual_speed(bit<64> d){
        hdr.cur.speed = d; 
    }

    table actual_speed_function{
        actions = {
            update_actual_speed;
            NoAction;
        }
        key = {
            hdr.cur.speed : ternary;
        }
        const default_action = NoAction;
        size = FUNCTION_SIZE;
    }

    action update_diff_speed(bit<64> d){
        eg_md.target_position = eg_md.target_position + d; 
    }

    table diff_speed_function{
        actions = {
            update_diff_speed;
            NoAction;
        }
        key = {
            eg_md.target_position: ternary;
        }
        const default_action = NoAction;
        size = FUNCTION_SIZE;
    }

    action add_target_speed(){
        hdr.cur.speed = hdr.cur.speed + eg_md.target_speed; 
    }

    action add_diff_speed(){
        hdr.cur.speed = hdr.cur.speed + eg_md.target_position; 
    }

    apply {
        if (hdr.cur.isValid()){
            #if ROBOT_ID_LEN == 8
                eg_md.robot_id = hdr.udp.srcPort[7:0];
            #else
                eg_md.robot_id = hdr.udp.srcPort & 16w0x0FFF;
            #endif

            bunny_e.apply();
            calculate_diff(); // in eg_md.target_position

            // *** apply functions
            target_speed_function.apply();
            actual_speed_function.apply();
            diff_speed_function.apply();

            // *** sum components
            add_target_speed();
            add_diff_speed();

            // *** apply speed limit
            speed_limit.apply();
            
            swap_ipv4_addresses();
            swap_udp_ports();
            clear_checksums();
        }
    }
}

control SwitchEgressDeparser(
        packet_out pkt,
        inout header_t hdr,
        in egress_metadata_t eg_md,
        in egress_intrinsic_metadata_for_deparser_t eg_dprsr_md) {

    Checksum() ipv4_checksum;

    apply {

        hdr.ipv4.hdrChecksum = ipv4_checksum.update({
            hdr.ipv4.version,
            hdr.ipv4.ihl,
            hdr.ipv4.diffserv,
            hdr.ipv4.totalLen,
            hdr.ipv4.identification,
            hdr.ipv4._reserved,
            hdr.ipv4.dont_fragment,
            hdr.ipv4.more_fragments,
            hdr.ipv4.fragOffset,
            hdr.ipv4.ttl,
            hdr.ipv4.protocol,
            //hdr.ipv4.hdrChecksum,
            hdr.ipv4.srcAddr,
            hdr.ipv4.dstAddr            
        });

        pkt.emit(hdr.ethernet);
        pkt.emit(hdr.ipv4);
        pkt.emit(hdr.udp);
        pkt.emit(hdr.ptp);
        pkt.emit(hdr.cur);
    }
}

Pipeline(SwitchIngressParser(),
         SwitchIngress(),
         SwitchIngressDeparser(),
         SwitchEgressParser(),
         SwitchEgress(),
         SwitchEgressDeparser()) pipe;

Switch(pipe) main;

