# include "ros2socketcan.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

int ros2socketcan::read_event(int fd, struct js_event *event)
{
    ssize_t bytes;
    bytes = read(fd, event, sizeof(*event));
    if (bytes == sizeof(*event))
        return 0;
    //Error, could not read full event.
    return -1;
}

size_t ros2socketcan::get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;
    return buttons;
}

size_t ros2socketcan::get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

size_t ros2socketcan::get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

double ros2socketcan::GetCommandThrottle(double val, double inc)
{
  val += inc;
  if (val > 1){
    val = 1;
  } else if (val < 0){
    val = 0;}
  return val;
}

double ros2socketcan::GetCommandSteer(double val, double inc) 
{
  val += inc;
  if (val > 250){
    val = 250;
  } else if (val < 0){
    val = 0;}
  return val;
}

/*****************************************************************************************************************/
/*                                                                                                               */
/* Traitement des données Can à envoyer "ID = 0x10" sur topic :/CAN/can0/receive_tricycle_cmd & /CAN/can0/transmit */
/*                                                                                                               */
/*****************************************************************************************************************/
void ros2socketcan::JoystickLoopThreadFunc()
{
    device = "/dev/input/js0";
    js = open(device, O_RDONLY);
    can_msgs::msg::Frame canmsg;
    can_msgs::msg::TricycleControlCmd controlcmd;
    canmsg.id = 0x10; 
    canmsg.dlc = 8; 
    canmsg.err = 0; 
    canmsg.rtr = 0; 
    canmsg.eff = 0;
    controlcmd.steer = (int)state_can_data.candata[5];
    controlcmd.brake = "OFF";  


    if (js == -1){
        perror("Thread not created. Could not open joystick");
    }else{
        printf("Thread created successfully. Open joystick device is successful.\n");
    }
    /* This loop will exit if the controller is unplugged. */
    while (read_event(js, &event) == 0)
    {
        switch (event.type)
        {
            case JS_EVENT_BUTTON:
                state_button.tab[event.number] = event.value;
                if(state_button.tab[0] == 1){                                                                   /*Acceleration*/
                    controlcmd.throttle = 0.0;
                    if(state_button.tab[5] == 1 && FLAGS_throttle_inc_delta < 1)
                        FLAGS_throttle_inc_delta += 0.1;
                    if(state_button.tab[4] == 1 && FLAGS_throttle_inc_delta > 0)
                        FLAGS_throttle_inc_delta -= 0.1;
                    controlcmd.throttle = GetCommandThrottle(controlcmd.throttle, FLAGS_throttle_inc_delta);
                    state_can_data.candata[0] = 0x01;
                    cmd_acc = ((FLAGS_throttle_inc_delta * 10) * 85) + 1706;
                    state_can_data.candata[7] = cmd_acc & 0xFF;
                    state_can_data.candata[6] = (cmd_acc & 0xFF00) >> 8;
                    //state_can_data.candata[7] = (int)controlcmd.throttle & 0xFF;
                    //state_can_data.candata[6] = ((int)controlcmd.throttle & 0xFF00) >> 8;
                }else if(state_button.tab[1] == 1){                                                             /*Freinage*/
                    var_brake = !var_brake;
                    if(var_brake == true){
                        controlcmd.brake = "ON";
                        state_can_data.candata[0] = 0x00;
                        state_can_data.candata[6] = 0x00;
                        state_can_data.candata[7] = 0x00;
                    }else{
                        controlcmd.brake = "OFF";
                        state_can_data.candata[0] = 0x02;
                    }
                }else if(state_button.tab[11] == 1){                                                            /*Direction*/
                    //controlcmd.steer = GetCommandSteer(controlcmd.steer, FLAGS_steering_inc_delta);
                    //state_can_data.candata[0] = 0x01;
                    //state_can_data.candata[5] = controlcmd.steer; 
                }else if(state_button.tab[12] == 1){
                    //controlcmd.steer = GetCommandSteer(controlcmd.steer, -FLAGS_steering_inc_delta);
                    //state_can_data.candata[0] = 0x01;
                    //state_can_data.candata[5] = controlcmd.steer;
                }

                for(int i=0; i<8; i++){
                    canmsg.data[i] = state_can_data.candata[i];
                }
                
                if(event.value ==1 ){
                    test_pub_->publish(canmsg);
                    tricycletransmitcmd_->publish(controlcmd);
                }
                break;
            case JS_EVENT_AXIS:
                /* Ignore init events. */
                axis = get_axis_state(&event, axes);
                if (axis < 3){
                    //printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
                    //printf("Axis %zu at (%6d)\n", axis, axes[axis].x);
                    cmd_sterr = (125*axes[axis].x)/(-32757);
                    if (cmd_sterr < 0 && cmd_sterr >= -250)
                    {
                        cmd_sterr += 125;
                        controlcmd.steer = cmd_sterr;
                        state_can_data.candata[5] = controlcmd.steer;
                        printf("Gauche\n");
                    } else if (cmd_sterr > 0 && cmd_sterr <= 250)
                    {
                        cmd_sterr += 125; 
                        controlcmd.steer = cmd_sterr;
                        state_can_data.candata[5] = controlcmd.steer;
                        printf("Droite\n");
                    }
                }

                state_can_data.candata[0] = 0x01;
                for(int i=0; i<8; i++){
                    canmsg.data[i] = state_can_data.candata[i];
                }
                test_pub_->publish(canmsg);
                tricycletransmitcmd_->publish(controlcmd);
                break;
            default:
                /* Ignore init events. */
                break;
        }
        fflush(stdout);
    }
    close(js);
}

ros2socketcan::ros2socketcan(std::string can_socket2): Node("ros2" + can_socket2), stream(ios), signals(ios, SIGINT, SIGTERM)
{

}

void ros2socketcan::Init(const char* can_socket)
{
    printf("Using can socket %s\n", can_socket);
    
    const char* canname = can_socket;
        
    topicname_receive 	<< "CAN/" << canname << "/" << "receive";
    topicname_transmit  << "CAN/" << canname << "/" << "transmit";
    topicname_transmitbis  << "CAN/" << canname << "/" << "transmit_tricycle_cmd";
    topicname_receivebis  << "CAN/" << canname << "/" << "receive_tricycle_status";
      
    rclcpp::executors::MultiThreadedExecutor exec;
    
    publisher_ 		= this->create_publisher<can_msgs::msg::Frame>(topicname_receive.str(), 10);
    tricyclepublisher_ 		= this->create_publisher<can_msgs::msg::TricycleStatus>(topicname_receivebis.str(), 10);
    test_pub_ 		= this->create_publisher<can_msgs::msg::Frame>(topicname_transmit.str(), 10);
    tricycletransmitcmd_    = this->create_publisher<can_msgs::msg::TricycleControlCmd>(topicname_transmitbis.str(), 10);
    subscription_ 	= this->create_subscription<can_msgs::msg::Frame>(topicname_transmit.str(), 10, std::bind(&ros2socketcan::CanPublisher, this, _1));
    
    strcpy(ifr.ifr_name, can_socket);
    ioctl(natsock, SIOCGIFINDEX, &ifr);
    
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if(bind(natsock,(struct sockaddr *)&addr,sizeof(addr))<0)
    {
        perror("Error in socket bind");
    }

    stream.assign(natsock);

    std::cout << "ROS2 to CAN-Bus topic:" << subscription_->get_topic_name() 	<< std::endl;
    std::cout << "ROS2 to CAN-Bus topic:" << tricycletransmitcmd_->get_topic_name() 	<< std::endl;
    std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name() 	<< std::endl;
    std::cout << "CAN-Bus to ROS2 topic:" << tricyclepublisher_->get_topic_name() 	<< std::endl;
    
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener, this,std::ref(rec_frame),std::ref(stream)));

    keyboard_thread_.reset(new std::thread([this] { JoystickLoopThreadFunc(); }));
    
    signals.async_wait(std::bind(&ros2socketcan::stop, this));
    
    boost::system::error_code ec;
    
    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    bt.detach();
    
    rclcpp::spin(shared_from_this());
}

void ros2socketcan::stop()
{
    printf("\nEnd of Listener Thread. Please press strg+c again to stop the whole program.\n");
    ios.stop();
    signals.clear();
}

ros2socketcan::~ros2socketcan()
{
    printf("\nEnd of Publisher Thread. \n");
}

void ros2socketcan::CanSend(const can_msgs::msg::Frame msg)
{
    struct can_frame frame1;
    
    frame1.can_id = msg.id;
    
    if (msg.eff == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_EFF_FLAG;
    }
    
    if (msg.err == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_ERR_FLAG;
    }
    
    if (msg.rtr == 1)
    {
        frame1.can_id  = frame1.can_id + CAN_RTR_FLAG;
    }
    
    frame1.can_dlc = msg.dlc;

    for(int i=0;i<(int)frame1.can_dlc;i++)
    {
        frame1.data[i] = msg.data[i];
    }
     
    printf("ROS2 ---> CAN | %x | %i | ", frame1.can_id, frame1.can_dlc);
    for(int j=0;j<(int)frame1.can_dlc;j++)
    {
        printf("%i ", frame1.data[j]);
    }
    printf("\n");
    
    stream.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)),std::bind(&ros2socketcan::CanSendConfirm, this));
}

void ros2socketcan::CanPublisher(const can_msgs::msg::Frame::SharedPtr msg)
{

    can_msgs::msg::Frame msg1;
    msg1.id  = msg->id;
    msg1.dlc = msg->dlc;
    msg1.eff = msg->eff;
    msg1.rtr = msg->rtr;
    msg1.err = msg->err;
    msg1.data= msg->data;

    CanSend(msg1);

}

void ros2socketcan::CanSendConfirm(void)
{
    //std::cout << "Message sent" << std::endl;
}

void ros2socketcan::CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream)
{
    can_msgs::msg::Frame frame;
    //can_msgs::msg::Frame canmsg;
    //can_msgs::msg::TricycleControlCmd controlcmd;
    //std::stringstream s[8];
    
    frame.id = rec_frame.can_id; 
    frame.dlc = int(rec_frame.can_dlc);
    

    /**************************************************************************************************************/
    /*                                                                                                            */
    /* Traitement des données Can tricycle reçu "ID = 0x99" et envoi sur topic :/CAN/can0/receive_tricycle_status */
    /*                                                                                                            */
    /**************************************************************************************************************/
    can_msgs::msg::TricycleStatus framebis;
    framebis.id = rec_frame.can_id;
    //framebis.dlc = int(rec_frame.can_dlc);

    if(framebis.id == 153)
    {
        std::string binary = std::bitset<8>(rec_frame.data[0]).to_string();

        /* Mode de conduite ===> Bits status1 (bit 6 : LOCAL si 1 / REMOTE 0) */
        if(binary[1] == '1'){
            framebis.tricycle_mode = "LOCAL";
        }else{
            framebis.tricycle_mode = "REMOTE";
        }
    
        /* Test de l'alimentation des microcontrolleurs ===> Bits status1 (bit 0 : alim  5V OK si 1)*/
        if(binary[7] == '1'){
            framebis.mcu_power_supply = "OK";
        }else{
            framebis.mcu_power_supply = "DEFAULT";
        }

        /* Test de l'alimentation du servo-moteur ===> Bits status1 (bit 0 : alim  14.8V OK si 1)*/
        if(binary[5] == '1'){
            framebis.steering_sys_voltage = "OK";
        }else{
            framebis.steering_sys_voltage = "DEFAULT";
        }

        /* Test de l'alimentation du vérin de freinage ===> Bits status1 (bit 0 : alim  14.8V OK si 1)*/
        if(binary[6] == '1'){
            framebis.brake_sys_voltage = "OK";
        }else{
            framebis.brake_sys_voltage = "DEFAULT";
        }

        /* Etat/position du vérin de freinage ===> Bits status1 (bit 0 : alim  14.8V OK si 1)*/
        if(binary[0] == '1'){
            framebis.brake_status = "ON";
        }else{
            framebis.brake_status = "OFF";
        }
    
        /*Pourcentage de la tension de la baterie (37.7v ---> min-54.6v ---> max)*/
        framebis.battery_voltage = ((rec_frame.data[1] * 256) + rec_frame.data[2])/10;  // tension en volt
        framebis.battery_pct = (framebis.battery_voltage * 100) / 54.6;
   
        /*Vitesse de roue calcule de coefficient: 1.585*0.001*3600 = 5.7*/
        float r= 0.25;
        float wheel_speed_rps = ((rec_frame.data[3] * 256) + rec_frame.data[4])/100;    // en tours/secondes
        framebis.wheel_speed = wheel_speed_rps * 5.7;                                   // en kilomètre/heure
        float wheel_speed_rpm = wheel_speed_rps*60;                                     // en tours/minutes
        float wheel_speed_kph = (wheel_speed_rpm*3*M_PI*r)/25 ;                         // en kilomètre/heure
        /*std::cout << wheel_speed_rps << std::endl;
        std::cout << wheel_speed_rpm << std::endl;
        std::cout << wheel_speed_kph << std::endl;*/
    }
    
    /***************************************************************************************/
    /*                                                                                     */
    /*Affichage des données reçu par CAN : ID en hex(%x), DLC en dec(%i) et Data en hex(%x)*/
    /*                                                                                     */
    /***************************************************************************************/
    printf("CAN <--- ROS2 | %x | %i | ", rec_frame.can_id, rec_frame.can_dlc);
    for(int i=0; i<rec_frame.can_dlc; i++){
         frame.data[i]=rec_frame.data[i];
         //s[i] << std::hex << rec_frame.data[i];
    }
    current_frame = frame;
    /*for(int i=0; i<rec_frame.can_dlc; i++)
        std::cout << s[i].str() << " | ";*/
    
    for(int j=0;j<(int)rec_frame.can_dlc;j++){
        printf("%x ", rec_frame.data[j]);
    }
    printf("\n"); 

    /*if(state_button.tab[11] == 1){
        printf("Button Gauche Appuyer\n");
        FLAGS_steering_inc_delta += 0.5;
    }*/

    /*********************************************/
    /*                                           */
    /* Publication des données sur les topics ros*/
    /*                                           */
    /*********************************************/

    publisher_->publish(frame);
    //test_pub_->publish(canmsg);
    tricyclepublisher_->publish(framebis);
    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),std::bind(&ros2socketcan::CanListener,this, std::ref(rec_frame),std::ref(stream)));
    
}

/*********************************************/
/*                                           */
/*              Fonction Main                */
/*                                           */
/*********************************************/

int main(int argc, char *argv[])
{
    std::cout << programdescr << std::endl;
    rclcpp::init(argc, argv);

    if (argc < 2) {
        auto ros2canptr = std::make_shared<ros2socketcan>();
        ros2canptr -> Init();
    }
    else{
        auto ros2canptr = std::make_shared<ros2socketcan>(argv[1]);
        ros2canptr -> Init(argv[1]);
    }
    
    return 0;
}
