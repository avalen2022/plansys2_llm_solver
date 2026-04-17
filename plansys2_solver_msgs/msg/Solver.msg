# Classification types
uint8 CORRECT=0
uint8 MODIFY_PLAN=1
uint8 MODIFY_DOMAIN=2
uint8 UNSOLVABLE=3
uint8 ERROR=4

float32 time
uint8 classification
string[] add_predicates
string[] remove_predicates
string[] add_instances
string[] domain_changes
string resolution
