python3 gen_speed_limit_entries.py > a_speed_limit_entries.p4
#python3 gen_diff_speed_function_entries.py > a_diff_speed_function_entries.p4


here=$(pwd)
cd /root/bf-sde-9.2.0/
source set_sde.bash
cd $here
/root/tools/p4_build.sh ur.p4
