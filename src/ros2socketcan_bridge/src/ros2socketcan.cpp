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

/**********************************************************************************************************************/
/*                                                                                                                    */
/* Traitement des données Can à envoyer "ID = 0x10" sur topic :/CAN/can0/manuel_tricycle_control & /CAN/can0/transmit */
/*                                                                                                                    */
/*  TRAME ID 0x10 :                                                                                                   */
/* -----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* |      8 bits/0     |      8 bits/1    |      8 bits/2     |      8 bits/3     |      8 bits/4     |      8 bits/5     |      8 bits/6     |      8 bits/7      |*/
/* |Bits commande cmd10|      pwm_frein H  |     pwm_frein L   |  force_frein H    |   force_frein L   |  angle direction  |   DAC_accel  H    |  DAC_accel L      |*/
/* -----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*                                                                                                                     */
/* struct of can data : candata[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};                                 */
/*                                                                                                                     */
/***********************************************************************************************************************/
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
    controlcmd.steer = (float)state_can_data.candata[5];
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
                /* @@ Acceleration [0;1] ===> button A (green button) + LB/RB
                @ La vitesse du tricycle a été brider à 20 km/h pour des raisons se sécurité en cas de défaince du système.
                @ De plus il y'a une valeur de seuil à laquelle répond le tricycle (avec des valeurs trop faibles aucun résultats)
                @ Chaque valeur de la commande throttle [0;1] aura une valeur correspondante en hexadécimal 

               1706    85     85     85     85       85       85       85       85       85       85      85   2558
                |------------------------------------------------------------------------------------------------>   
               6AA                                                                                              9FE 
                */
                if(state_button.tab[0] == 1 && controlcmd.brake != "ON"){                                                                  
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
                }else if(state_button.tab[1] == 1){     
                /* @@Freinage ON/OFF ===> Button B (red button)
                @ La fonction de freinage est assurée par la touche B du joystick.
                @ Il actionne le verin de freinage pour imobiliser le tricycle mais garde en memoire la valeur de l'acc...
                */
                    var_brake = !var_brake;
                    if(var_brake == true && controlcmd.brake != "ON"){
                        controlcmd.brake = "ON";
                        state_can_data.candata[0] = 0x00;
                        state_can_data.candata[6] = 0x00;
                        state_can_data.candata[7] = 0x00;
                    }else{
                        controlcmd.brake = "OFF";
                        state_can_data.candata[0] = 0x02;
                    }
                }else if(state_button.tab[8] == 1){ 
                /* @@Freinage d'urgence ===> Guide button
                @ Arrêt d'urgence en cas d'incident et remise à 0 de la valeur de l'accéleration 
                */
                    var_brake = true;
                    controlcmd.throttle = 0.0;                                                        
                    controlcmd.brake = "ON";
                    FLAGS_throttle_inc_delta = 0.0;
                    state_can_data.candata[0] = 0x00;
                    state_can_data.candata[6] = 0x00;
                    state_can_data.candata[7] = 0x00;
                }
                for(int i=0; i<8; i++)
                    canmsg.data[i] = state_can_data.candata[i];

                if(event.value ==1){
                    test_pub_->publish(canmsg);
                    tricycletransmitcmd_->publish(controlcmd);
                }
                break;
            case JS_EVENT_AXIS:
                axis = get_axis_state(&event, axes); 
                /* @@Direction [-1;1] ===> Left stick
                @ La direction est controler par le Stick gauche . 
                @ angle direction  [0 ... 125 (0x7D) ...250 (0xFA)] --> [ 0° (droite) ... 60° (centre) ... 120° (gauche)]
                @ X £ [0;250] et Y £ [-1;1] <===> Y = aX + b  ===> Y = (-1/125) * X + 1 
                */                                                          
                if (axis < 3 && axis == 0){
                    cmd_sterr = (125*axes[axis].x)/(-32757);
                    if (cmd_sterr < 0 && cmd_sterr >= -250){
                        cmd_sterr += 125;
                        cmd_sterr1 = (-0.008 * (float)cmd_sterr) + 1;
                        controlcmd.steer = cmd_sterr1;
                        state_can_data.candata[5] = cmd_sterr;
                    } else if (cmd_sterr > 0 && cmd_sterr <= 250){
                        cmd_sterr += 125; 
                        cmd_sterr1 = (-0.008 * (float)cmd_sterr) + 1;
                        controlcmd.steer = cmd_sterr1;
                        state_can_data.candata[5] = cmd_sterr;
                    }

                    state_can_data.candata[0] = 0x01;
                    for(int i=0; i<8; i++)
                        canmsg.data[i] = state_can_data.candata[i];

                    test_pub_->publish(canmsg);
                    tricycletransmitcmd_->publish(controlcmd);
                }
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
    topicname_transmitbis  << "/" << "manuel_car_control";
    topicname_receivebis   << "/" << "system_check";

      
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

/*************************************************************************************************************************/
/*                                                                                                                       */
/* Traitement des données Can reçu "ID = 0x99" et envoi sur topic :/CAN/can0/system_check & /CAN/can0/receive */
/*                                                                                                                       */
/*************************************************************************************************************************/
void ros2socketcan::CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream)
{
    can_msgs::msg::Frame frame;
    frame.id = rec_frame.can_id; 
    frame.dlc = int(rec_frame.can_dlc);
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
        framebis.battery_voltage = ((rec_frame.data[1] * 256) + rec_frame.data[2])/10;      // tension en volt
        framebis.battery_pct = (framebis.battery_voltage * 100) / 54.6;
   
        /*Vitesse de roue calcule de coefficient: 1.585*0.001*3600 = 5.7*/
        //float r= 0.25;
        float wheel_speed_rps = ((rec_frame.data[3] * 256) + rec_frame.data[4])/100;        // en tours/secondes
        //framebis.wheel_speed = wheel_speed_rps * 5.7;                                       // en kilomètre/heure
        framebis.wheel_speed = round(wheel_speed_rps * 5.7);
        /*float wheel_speed_rpm = wheel_speed_rps*60;                                       // en tours/minutes
        float wheel_speed_kph = (wheel_speed_rpm*3*M_PI*r)/25 ;                             // en kilomètre/heure
        std::cout << wheel_speed_rps << std::endl;
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
    }
    current_frame = frame;
    
    for(int j=0;j<(int)rec_frame.can_dlc;j++){
        printf("%x ", rec_frame.data[j]);
    }
    printf("\n"); 

    /*********************************************/
    /*                                           */
    /* Publication des données sur les topics ros*/
    /*                                           */
    /*********************************************/

    publisher_->publish(frame);
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
