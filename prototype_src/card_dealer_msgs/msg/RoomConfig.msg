# File: src/card_dealer_msgs/msg/RoomConfig.msg

string room_code
string room_status
string timestamp
int32 num_players
int32 card_gain
int32 starting_card

# This refers to the Player.msg defined above
Player[] players
