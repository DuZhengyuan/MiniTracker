#include <iostream>
#include <algorithm>
#include <fstream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <Winsock2.h>
#include <winsock.h>
#include <pthread.h>
#include <windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}


#define  MAX_BUF_SIZE 1024
#define RAD_TO_DEG (180/M_PI)
#define DEG_TO_RAD (M_PI/180)
#define ARDRONE_SESSION_ID          "d2e081a3"      // SessionID
#define ARDRONE_PROFILE_ID          "be27e2e4"      // Profile ID
#define ARDRONE_APPLOCATION_ID      "d87f7e0c"      // Application ID
#define msleep(ms) Sleep((DWORD)ms)
#define CV_VK_UP (VK_UP<<16)
#define CV_VK_DOWN (VK_DOWN<<16)
#define CV_VK_LEFT (VK_LEFT<<16)
#define CV_VK_RIGHT (VK_RIGHT<<16)

// State masks
enum ARDRONE_STATE_MASK {
	ARDRONE_FLY_MASK = 1U << 0, // FLY MASK                  : (0) Ardrone is landed, (1) Ardrone is flying
	ARDRONE_VIDEO_MASK = 1U << 1, // VIDEO MASK                : (0) Video disable, (1) Video enable
	ARDRONE_VISION_MASK = 1U << 2, // VISION MASK               : (0) Vision disable, (1) Vision enable
	ARDRONE_CONTROL_MASK = 1U << 3, // CONTROL ALGO              : (0) Euler angles control, (1) Angular speed control
	ARDRONE_ALTITUDE_MASK = 1U << 4, // ALTITUDE CONTROL ALGO     : (0) Altitude control inactive (1) Altitude control active
	ARDRONE_USER_FEEDBACK_START = 1U << 5, // USER feedback             :     Start button state 
	ARDRONE_COMMAND_MASK = 1U << 6, // Control command ACK       : (0) None, (1) One received
	ARDRONE_CAMERA_MASK = 1U << 7, // CAMERA MASK               : (0) Camera not ready, (1) Camera ready
	ARDRONE_TRAVELLING_MASK = 1U << 8, // Travelling mask           : (0) Disable, (1) Enable
	ARDRONE_USB_MASK = 1U << 9, // USB key                   : (0) Usb key not ready, (1) Usb key ready
	ARDRONE_NAVDATA_DEMO_MASK = 1U << 10, // Navdata demo              : (0) All navdata, (1) Only navdata demo
	ARDRONE_NAVDATA_BOOTSTRAP = 1U << 11, // Navdata bootstrap         : (0) Options sent in all or demo mode, (1) No navdata options sent
	ARDRONE_MOTORS_MASK = 1U << 12, // Motors status             : (0) Ok, (1) Motors problem
	ARDRONE_COM_LOST_MASK = 1U << 13, // Communication Lost        : (1) Com problem, (0) Com is ok
	ARDRONE_VBAT_LOW = 1U << 15, // VBat low                  : (1) Too low, (0) Ok
	ARDRONE_USER_EL = 1U << 16, // User Emergency Landing    : (1) User EL is ON, (0) User EL is OFF
	ARDRONE_TIMER_ELAPSED = 1U << 17, // Timer elapsed             : (1) Elapsed, (0) Not elapsed
	ARDRONE_ANGLES_OUT_OF_RANGE = 1U << 19, // Angles                    : (0) Ok, (1) Out of range
	ARDRONE_ULTRASOUND_MASK = 1U << 21, // Ultrasonic sensor         : (0) Ok, (1) Deaf
	ARDRONE_CUTOUT_MASK = 1U << 22, // Cutout system detection   : (0) Not detected, (1) Detected
	ARDRONE_PIC_VERSION_MASK = 1U << 23, // PIC Version number OK     : (0) A bad version number, (1) Version number is OK */
	ARDRONE_ATCODEC_THREAD_ON = 1U << 24, // ATCodec thread ON         : (0) Thread OFF (1) thread ON
	ARDRONE_NAVDATA_THREAD_ON = 1U << 25, // Navdata thread ON         : (0) Thread OFF (1) thread ON
	ARDRONE_VIDEO_THREAD_ON = 1U << 26, // Video thread ON           : (0) Thread OFF (1) thread ON
	ARDRONE_ACQ_THREAD_ON = 1U << 27, // Acquisition thread ON     : (0) Thread OFF (1) thread ON
	ARDRONE_CTRL_WATCHDOG_MASK = 1U << 28, // CTRL watchdog             : (1) Delay in control execution (> 5ms), (0) Control is well scheduled
	ARDRONE_ADC_WATCHDOG_MASK = 1U << 29, // ADC Watchdog              : (1) Delay in uart2 dsr (> 5ms), (0) Uart2 is good
	ARDRONE_COM_WATCHDOG_MASK = 1U << 30, // Communication Watchdog    : (1) Com problem, (0) Com is ok
	ARDRONE_EMERGENCY_MASK = 1U << 31  // Emergency landing         : (0) No emergency, (1) Emergency
};

// Navdata tags
enum ARDRONE_NAVDATA_TAG {
	ARDRONE_NAVDATA_DEMO_TAG = 0,
	ARDRONE_NAVDATA_TIME_TAG = 1,
	ARDRONE_NAVDATA_RAW_MEASURES_TAG = 2,
	ARDRONE_NAVDATA_PHYS_MEASURES_TAG = 3,
	ARDRONE_NAVDATA_GYROS_OFFSETS_TAG = 4,
	ARDRONE_NAVDATA_EULER_ANGLES_TAG = 5,
	ARDRONE_NAVDATA_REFERENCES_TAG = 6,
	ARDRONE_NAVDATA_TRIMS_TAG = 7,
	ARDRONE_NAVDATA_RC_REFERENCES_TAG = 8,
	ARDRONE_NAVDATA_PWM_TAG = 9,
	ARDRONE_NAVDATA_ALTITUDE_TAG = 10,
	ARDRONE_NAVDATA_VISION_RAW_TAG = 11,
	ARDRONE_NAVDATA_VISION_OF_TAG = 12,
	ARDRONE_NAVDATA_VISION_TAG = 13,
	ARDRONE_NAVDATA_VISION_PERF_TAG = 14,
	ARDRONE_NAVDATA_TRACKERS_SEND_TAG = 15,
	ARDRONE_NAVDATA_VISION_DETECT_TAG = 16,
	ARDRONE_NAVDATA_WATCHDOG_TAG = 17,
	ARDRONE_NAVDATA_IPHONE_ANGLES_TAG = 18,
	ARDRONE_NAVDATA_ADC_DATA_FRAME_TAG = 18,
	ARDRONE_NAVDATA_VIDEO_STREAM_TAG = 19,
	ARDRONE_NAVDATA_GAME_TAG = 20,       // AR.Drone 1.7.4
	ARDRONE_NAVDATA_PRESSURE_RAW_TAG = 21,       // AR.Drone 2.0
	ARDRONE_NAVDATA_MAGNETO_TAG = 22,       // AR.Drone 2.0
	ARDRONE_NAVDATA_WIND_TAG = 23,       // AR.Drone 2.0
	ARDRONE_NAVDATA_KALMAN_PRESSURE_TAG = 24,       // AR.Drone 2.0
	ARDRONE_NAVDATA_HDVIDEO_STREAM_TAG = 25,       // AR.Drone 2.0
	ARDRONE_NAVDATA_WIFI_TAG = 26,       // AR.Drone 2.0
	ARDRONE_NAVDATA_ZIMMU3000_TAG = 27,       // AR.Drone 2.0
	ARDRONE_NAVDATA_GPS_TAG = 27,       // AR.Drone 2.4.1
	ARDRONE_NAVDATA_CKS_TAG = 0xFFFF
};


using namespace std;
//SPEED
struct SPEED{
	double vx, vy, vz, vr;
	double LR, FB, UD, Turn;
//------------------------------------------------------------------------------------------
	SPEED(double lr = 0.3, double fb = 0.3, double ud = 0.3, double tturn = 0.4){
//------------------------------------------------------------------------------------------
		LR = lr;
		FB = fb;
		UD = ud;
		Turn = tturn;
		vx = vy = vz = vr = 0;
	}
	void ZERO(){
		vx = vy = vz = vr = 0;
	}
	void Print(){
		printf("%.2f,%.2f,%.2f,%.2f\n", vx, vy, vz, vr);
	}
	void Update(double x, double y, double z){
		vx = x; vy = y; vz = z;
	}
	void Update(int bits, bool GG_T){//FBLRUD
		double rate = 1;
		for (int i = 0; i < 3; i++){  // if GG_T = false as pre; else Turn! @@@
			if (bits&(1 << (i * 2))){
				rate = 1;
			}
			else if (bits&(1 << (i * 2 + 1))){
				rate = -1;
			}
			else{
				rate = 0;
			}
			if (i == 0){
				vx = rate*FB;
			}
			else if (i == 1){
				if (GG_T == false)vy = rate*LR;
				else vr = rate*Turn;
			}
			else{
				vz = rate*UD;
			}
		}
		if (GG_T == false)vr = 0;
	}
	void Roll(double add){
		vr = add;
	}
	void Up_FB(bool f = true){
		if (f) vx = FB;
		else vx = -FB;
	}
	void Up_LR(bool f = true){
		if (f) vy = LR;
		else vy = -LR;
	}
	bool operator == (const SPEED x)const {
		return (x.vx) / std::abs(x.vx) == vx / std::abs(vx) && (x.vy) / std::abs(x.vy) == vy / std::abs(vy) && x.vz / std::abs(x.vz) == vz / std::abs(vz) && x.vr == vr;
	}
	bool isEmpty(){
		return vx == 0 && vy == 0 && vz == 0 && vr == 0;
	}
	SPEED GotSlow(){
		SPEED ans;
		ans.Update(vx / 4, vy / 4, vz / 4);
		return ans;
	}
	SPEED GotBig(){
		SPEED ans;
		ans.Update(vx * 3, vy * 3, vz * 3);
		return ans;
	}
	void Lock(int bits=0){
		if (bits ==0 && vz != 0)vx = vr = vy = 0;
		if (bits == 0 && vx != 0) vy = vz = vr = 0;
		if (bits == 0 && (vy != 0 || vr != 0))vx = vz = 0;
		if (bits == 1) vy = vr = vz = 0;
		if (bits == 2)vx = vr = vz = 0;
		if (bits == 3)vx = vr = vy = 0;
		if (bits == 4){
			vx = 0;
			if (vz != 0)vy = vr = 0;
			//if (vy != 0 || vr != 0) vz = 0;
		}
	}
};
// UDP Class Socket
class UDPSocket {
public:
	UDPSocket();                            // Constructor
	virtual ~UDPSocket();                   // Destructor
	int  open(const char *addr, int port);  // Initialize
	int  send2(void *data, size_t size);    // Send data
	int  sendf(const char *str, ...);       // Send with format
	int  receive(void *data, size_t size);  // Receive data
	void close(void);                       // Finalize
private:
	SOCKET sock;                            // Socket
	sockaddr_in server_addr, client_addr;   // Server/Client IP adrress
};


// --------------------------------------------------------------------------
// UDPSocket::UDPSocket()
// Description : Constructor of UDPSocket class.
// --------------------------------------------------------------------------
UDPSocket::UDPSocket()
{
	sock = INVALID_SOCKET;
}

// --------------------------------------------------------------------------
// UDPSocket::~UDPSocket()
// Description : Destructor of UDPSocket class.
// --------------------------------------------------------------------------
UDPSocket::~UDPSocket()
{
	close();
}

// --------------------------------------------------------------------------
// UDPSocket::open(IP address, Port number)
// Description  : Initialize specified  socket.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int UDPSocket::open(const char *addr, int port)
{
	// Initialize WSA
	WSAData wsaData;
	WSAStartup(MAKEWORD(1, 1), &wsaData);

	// Create a socket
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock == INVALID_SOCKET) {
		//printf("ERROR: socket() failed. (%s, %d)\n", __FILE__, __LINE__);
		return 0;
	}

	// Set the port and address of server
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons((u_short)port);
	server_addr.sin_addr.s_addr = inet_addr(addr);

	// Set the port and address of client
	memset(&client_addr, 0, sizeof(client_addr));
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(0);
	client_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	// Bind the socket
	if (bind(sock, (sockaddr*)&client_addr, sizeof(client_addr)) == SOCKET_ERROR) {
		//printf("ERROR: bind() failed. (%s, %d)\n", __FILE__, __LINE__);
		return 0;
	}

	//// Set to non-blocking mode
	//#if _WIN32
	//u_long nonblock = 1;
	//if (ioctlsocket(sock, FIONBIO, &nonblock) == SOCKET_ERROR) {
	//    //printf("ERROR: ioctlsocket() failed. (%s, %d)\n", __FILE__, __LINE__);  
	//    return 0;
	//}
	//#else
	//int flag = fcntl(sock, F_GETFL, 0);
	//if (flag < 0) {
	//    //printf("ERROR: fcntl(F_GETFL) failed. (%s, %d)\n", __FILE__, __LINE__);  
	//    return 0;
	//}
	//if (fcntl(sock, F_SETFL, flag|O_NONBLOCK) < 0) {
	//    //printf("ERROR: fcntl(F_SETFL) failed. (%s, %d)\n", __FILE__, __LINE__);  
	//    return 0;
	//}
	//#endif

	// Enable re-use address option
	int reuse = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) == SOCKET_ERROR) {
		//printf("ERROR: setsockopt() failed. (%s, %d)\n", __FILE__, __LINE__);
		return 0;
	}

	return 1;
}

// --------------------------------------------------------------------------
// UDPSocket:::send2(Sending data, Size of data)
// Description  : Send the specified data.
// Return value : SUCCESS: Number of sent bytes  FAILURE: 0
// --------------------------------------------------------------------------
int UDPSocket::send2(void *data, size_t size)
{
	// The socket is invalid
	if (sock == INVALID_SOCKET) return 0;

	// Send data
	int n = (int)sendto(sock, (char*)data, size, 0, (sockaddr*)&server_addr, sizeof(server_addr));
	if (n < 1) return 0;

	return n;
}

// --------------------------------------------------------------------------
// UDPSocket::sendf(Messages)
// Description  : Send the data with format.
// Return value : SUCCESS: Number of sent bytes  FAILURE: 0
// --------------------------------------------------------------------------
int UDPSocket::sendf(const char *str, ...)
{
	char msg[1024];

	// The socket is invalid
	if (sock == INVALID_SOCKET) return 0;

	// Apply format 
	va_list arg;
	va_start(arg, str);
	vsnprintf_s(msg, 1024,1024, str, arg);
	va_end(arg);

	// Send data
	return send2(msg, (int)strlen(msg) + 1);
}

// --------------------------------------------------------------------------
// UDPSocket::receive(Receiving data, Size of data)
// Description  : Receive the data.
// Return value : SUCCESS: Number of received bytes  FAILURE: 0
// --------------------------------------------------------------------------
int UDPSocket::receive(void *data, size_t size)
{
	// The socket is invalid.
	if (sock == INVALID_SOCKET) return 0;

	// Receive data
	sockaddr_in addr;
	int len = sizeof(addr);
	int n = (int)recvfrom(sock, (char*)data, size, 0, (sockaddr*)&addr, &len);
	if (n < 1) return 0;

	// Server has the same IP address of client
	//if (addr.sin_addr.S_un.S_addr != server_addr.sin_addr.S_un.S_addr) return 0;

	return n;
}

// --------------------------------------------------------------------------
// UDPSocket::close()
// Description  : Finalize the socket.
// Return value : NONE
// --------------------------------------------------------------------------
void UDPSocket::close(void)
{
	// Close the socket
	if (sock != INVALID_SOCKET) {
		closesocket(sock);
		sock = INVALID_SOCKET;
	}
	// Finalize WSA
	WSACleanup();
}



struct NAVDATA {
	// 3x3 matrix
	struct matrix33_t {
		float m11, m12, m13;
		float m21, m22, m23;
		float m31, m32, m33;
	};

	// 3x1 vector
	union vector31_t {
		float v[3];
		struct {
			float x;
			float y;
			float z;
		};
	};


	// 2x1 vector
	union vector21_t {
		float v[2];
		struct {
			float x;
			float y;
		};
	};

	// Velocities
	struct velocities_t {
		float x;
		float y;
		float z;
	};

	// Screen point
	struct screen_point_t {
		int x;
		int y;
	};

	// Header
	unsigned int header;
	unsigned int ardrone_state;
	unsigned int sequence;
	unsigned int vision_defined;

	// Demo
	struct NAVDATA_DEMO {
		unsigned short tag;
		unsigned short size;
		unsigned int   ctrl_state;
		unsigned int   vbat_flying_percentage;
		float          theta;
		float          phi;
		float          psi;
		int            altitude;
		float          vx;
		float          vy;
		float          vz;
		unsigned int   num_frames;                // Don't use
		matrix33_t     detection_camera_rot;      // Don't use
		vector31_t     detection_camera_trans;    // Don't use
		unsigned int   detection_tag_index;       // Don't use
		unsigned int   detection_camera_type;     // Don't use
		matrix33_t     drone_camera_rot;          // Don't use
		vector31_t     drone_camera_trans;        // Don't use
	} demo;

	// Timestamp
	struct NAVDATA_TIME {
		unsigned short tag;
		unsigned short size;
		unsigned int   time;
	} time;

	// Raw measurements
	struct NAVDATA_RAW_MEASURES {
		unsigned short tag;
		unsigned short size;
		unsigned short raw_accs[3];         // filtered accelerometers
		short          raw_gyros[3];        // filtered gyrometers
		short          raw_gyros_110[2];    // gyrometers  x/y 110 deg/s
		unsigned int   vbat_raw;            // battery voltage raw (mV)
		unsigned short us_debut_echo;
		unsigned short us_fin_echo;
		unsigned short us_association_echo;
		unsigned short us_distance_echo;
		unsigned short us_courbe_temps;
		unsigned short us_courbe_valeur;
		unsigned short us_courbe_ref;
		unsigned short flag_echo_ini;
		//unsigned short frame_number;
		unsigned short nb_echo;
		unsigned int   sum_echo;
		int            alt_temp_raw;
		short          gradient;
	} raw_measures;

	// Physical measurements
	struct NAVDATA_PHYS_MEASURES {
		unsigned short tag;
		unsigned short size;
		float          accs_temp;
		unsigned short gyro_temp;
		float          phys_accs[3];
		float          phys_gyros[3];
		unsigned int   alim3V3;         // 3.3 volt alim       [LSB]
		unsigned int   vrefEpson;       // ref volt Epson gyro [LSB]
		unsigned int   vrefIDG;         // ref volt IDG gyro   [LSB]
	} phys_measures;

	// Gyros offsets
	struct NAVDATA_GYROS_OFFSETS {
		unsigned short tag;
		unsigned short size;
		float          offset_g[3];
	} gyros_offsets;

	// Euler angles
	struct NAVDATA_EULER_ANGLES {
		unsigned short tag;
		unsigned short size;
		float          theta_a;
		float          phi_a;
	} euler_angles;

	// References
	struct NAVDATA_REFERENCES {
		unsigned short tag;
		unsigned short size;
		int            ref_theta;
		int            ref_phi;
		int            ref_theta_I;
		int            ref_phi_I;
		int            ref_pitch;
		int            ref_roll;
		int            ref_yaw;
		int            ref_psi;
		float          vx_ref;
		float          vy_ref;
		float          theta_mod;
		float          phi_mod;
		float          k_v_x;
		float          k_v_y;
		unsigned int   k_mode;
		float          ui_time;
		float          ui_theta;
		float          ui_phi;
		float          ui_psi;
		float          ui_psi_accuracy;
		int            ui_seq;
	} references;

	// Trims
	struct NAVDATA_TRIMS {
		unsigned short tag;
		unsigned short size;
		float          angular_rates_trim_r;
		float          euler_angles_trim_theta;
		float          euler_angles_trim_phi;
	} trims;

	// RC references
	struct NAVDATA_RC_REFERENCES {
		unsigned short tag;
		unsigned short size;
		int            rc_ref_pitch;
		int            rc_ref_roll;
		int            rc_ref_yaw;
		int            rc_ref_gaz;
		int            rc_ref_ag;
	} rc_references;

	// PWM
	struct NAVDATA_PWM {
		unsigned short tag;
		unsigned short size;
		unsigned char  motor1;
		unsigned char  motor2;
		unsigned char  motor3;
		unsigned char  motor4;
		unsigned char  sat_motor1;
		unsigned char  sat_motor2;
		unsigned char  sat_motor3;
		unsigned char  sat_motor4;
		float          gaz_feed_forward;
		float          gaz_altitude;
		float          altitude_integral;
		float          vz_ref;
		int            u_pitch;
		int            u_roll;
		int            u_yaw;
		float          yaw_u_I;
		int            u_pitch_planif;
		int            u_roll_planif;
		int            u_yaw_planif;
		float          u_gaz_planif;
		unsigned short current_motor1;
		unsigned short current_motor2;
		unsigned short current_motor3;
		unsigned short current_motor4;
		float          altitude_prop;
		float          altitude_der;
	} pwm;

	// Altitude
	struct NAVDATA_ALTITUDE {
		unsigned short tag;
		unsigned short size;
		int            altitude_vision;
		float          altitude_vz;
		int            altitude_ref;
		int            altitude_raw;
		float          obs_accZ;
		float          obs_alt;
		vector31_t     obs_x;
		unsigned int   obs_state;
		vector21_t     est_vb;
		unsigned int   est_state;
	} altitude;

	// Vision (raw)
	struct NAVDATA_VISION_RAW {
		unsigned short tag;
		unsigned short size;
		float          vision_tx_raw;
		float          vision_ty_raw;
		float          vision_tz_raw;
	} vision_raw;

	// Vision (offset?)
	struct NAVDATA_VISION_OF {
		unsigned short tag;
		unsigned short size;
		float          of_dx[5];
		float          of_dy[5];
	} vision_of;

	// Vision
	struct NAVDATA_VISION {
		unsigned short tag;
		unsigned short size;
		unsigned int   vision_state;
		int            vision_misc;
		float          vision_phi_trim;
		float          vision_phi_ref_prop;
		float          vision_theta_trim;
		float          vision_theta_ref_prop;
		int            new_raw_picture;
		float          theta_capture;
		float          phi_capture;
		float          psi_capture;
		int            altitude_capture;
		unsigned int   time_capture;    // time in TSECDEC format (see config.h)
		velocities_t   body_v;
		float          delta_phi;
		float          delta_theta;
		float          delta_psi;
		unsigned int   gold_defined;
		unsigned int   gold_reset;
		float          gold_x;
		float          gold_y;
	} vision;

	// Vision performances
	struct NAVDATA_VISION_PERF {
		unsigned short tag;
		unsigned short size;
		float          time_szo;
		float          time_corners;
		float          time_compute;
		float          time_tracking;
		float          time_trans;
		float          time_update;
		float          time_custom[20];
	} vision_perf;

	// Trackers
	struct NAVDATA_TRACKERS_SEND {
		unsigned short tag;
		unsigned short size;
		int            locked[30];
		screen_point_t point[30];
	} trackers_send;

	// Vision detection
	struct NAVDATA_VISION_DETECT {
		unsigned short tag;
		unsigned short size;
		unsigned int   nb_detected;
		unsigned int   type[4];
		unsigned int   xc[4];
		unsigned int   yc[4];
		unsigned int   width[4];
		unsigned int   height[4];
		unsigned int   dist[4];
		float          orientation_angle[4];
		matrix33_t     rotation[4];
		vector31_t     translation[4];
		unsigned int   camera_source[4];
	} vision_detect;

	// Watchdog
	struct NAVDATA_WATCHDOG {
		unsigned short tag;
		unsigned short size;
		int            watchdog;
	} watchdog;

	// ADC data
	struct NAVDATA_ADC_DATA_FRAME {
		unsigned short tag;
		unsigned short size;
		unsigned int   version;
		unsigned char  data_frame[32];
	} adc_data_frame;

	// Video stream
	struct NAVDATA_VIDEO_STREAM {
		unsigned short tag;
		unsigned short size;
		unsigned char  quant;               // quantizer reference used to encode frame [1:31]
		unsigned int   frame_size;          // frame size (bytes)
		unsigned int   frame_number;        // frame index
		unsigned int   atcmd_ref_seq;       // atmcd ref sequence number
		unsigned int   atcmd_mean_ref_gap;  // mean time between two consecutive atcmd_ref (ms)
		float          atcmd_var_ref_gap;
		unsigned int   atcmd_ref_quality;   // estimator of atcmd link quality

		// drone2
		unsigned int   out_bitrate;         // measured out throughput from the video tcp socket
		unsigned int   desired_bitrate;     // last frame size generated by the video encoder
		int            data1;
		int            data2;
		int            data3;
		int            data4;
		int            data5;
		unsigned int   tcp_queue_level;
		unsigned int   fifo_queue_level;
	} video_stream;

	// Games
	struct NAVDATA_GAMES {
		unsigned short tag;
		unsigned short size;
		unsigned int   double_tap_counter;
		unsigned int   finish_line_counter;
	} games;

	// Preassure (raw)
	struct NAVDATA_PRESSURE_RAW {
		unsigned short tag;
		unsigned short size;
		unsigned int   up;
		unsigned short ut;
		unsigned int   temperature_meas;
		unsigned int   pression_meas;
	} pressure_raw;

	// Magneto
	struct NAVDATA_MAGNETO {
		unsigned short tag;
		unsigned short size;
		short          mx;
		short          my;
		short          mz;
		vector31_t     magneto_raw;             // magneto in the body frame, in mG
		vector31_t     magneto_rectified;
		vector31_t     magneto_offset;
		float          heading_unwrapped;
		float          heading_gyro_unwrapped;
		float          heading_fusion_unwrapped;
		char           magneto_calibration_ok;
		unsigned int   magneto_state;
		float          magneto_radius;
		float          error_mean;
		float          error_var;
		float          tmp1, tmp2;              // dummy ?
	} magneto;

	// Wind
	struct NAVDATA_WIND {
		unsigned short tag;
		unsigned short size;
		float          wind_speed;              // estimated wind speed [m/s]
		float          wind_angle;              // estimated wind direction in North-East frame [rad] e.g. if wind_angle is pi/4, wind is from South-West to North-East
		float          wind_compensation_theta;
		float          wind_compensation_phi;
		float          state_x1;
		float          state_x2;
		float          state_x3;
		float          state_x4;
		float          state_x5;
		float          state_x6;
		float          magneto_debug1;
		float          magneto_debug2;
		float          magneto_debug3;
	} wind;

	// Kalman filter
	struct NAVDATA_KALMAN_PRESSURE {
		unsigned short tag;
		unsigned short size;
		float          offset_pressure;
		float          est_z;
		float          est_zdot;
		float          est_bias_PWM;
		float          est_biais_pression;
		float          offset_US;
		float          prediction_US;
		float          cov_alt;
		float          cov_PWM;
		float          cov_vitesse;
		bool           bool_effet_sol;
		float          somme_inno;
		bool           flag_rejet_US;
		float          u_multisinus;
		float          gaz_altitude;
		bool           flag_multisinus;
		bool           flag_multisinus_debut;
	} kalman_pressure;

	// HD video stream
	struct NAVDATA_HDVIDEO_STREAM {
		unsigned short tag;
		unsigned short size;
		unsigned int   hdvideo_state;
		unsigned int   storage_fifo_nb_packets;
		unsigned int   storage_fifo_size;
		unsigned int   usbkey_size;           // USB key in kbytes - 0 if no key present
		unsigned int   usbkey_freespace;      // USB key free space in kbytes - 0 if no key present
		unsigned int   frame_number;          // 'frame_number' PaVE field of the frame starting to be encoded for the HD stream
		unsigned int   usbkey_remaining_time; // time in seconds
	} hdvideo_stream;

	// WiFi
	struct NAVDATA_WIFI {
		unsigned short tag;
		unsigned short size;
		unsigned int   link_quality;
	} wifi;

	// Zimmu 3000
	struct NAVDATA_ZIMMU_3000 {
		unsigned short tag;
		unsigned short size;
		int            vzimmuLSB;
		float          vzfind;
	} zimmu_3000;

	// GPS (for AR.Drone 2.4.1, or later)
	struct NAVDATA_GPS {
		unsigned short tag;                  /*!< Navdata block ('option') identifier */
		unsigned short size;                 /*!< set this to the size of this structure */
		double         lat;                  /*!< Latitude */
		double         lon;                  /*!< Longitude */
		double         elevation;            /*!< Elevation */
		double         hdop;                 /*!< hdop */
		int            data_available;       /*!< When there is data available */
		unsigned char  unk_0[8];
		double         lat0;                 /*!< Latitude ??? */
		double         lon0;                 /*!< Longitude ??? */
		double         lat_fuse;             /*!< Latitude fused */
		double         lon_fuse;             /*!< Longitude fused */
		unsigned int   gps_state;            /*!< State of the GPS, still need to figure out */
		unsigned char  unk_1[40];
		double         vdop;                 /*!< vdop */
		double         pdop;                 /*!< pdop */
		float          speed;                /*!< speed */
		unsigned int   last_frame_timestamp; /*!< Timestamp from the last frame */
		float          degree;               /*!< Degree */
		float          degree_mag;           /*!< Degree of the magnetic */
		unsigned char  unk_2[16];
		struct {
			unsigned char sat;
			unsigned char cn0;
		} channels[12];
		int             gps_plugged;         /*!< When the gps is plugged */
		unsigned char   unk_3[108];
		double          gps_time;            /*!< The gps time of week */
		unsigned short  week;                /*!< The gps week */
		unsigned char   gps_fix;             /*!< The gps fix */
		unsigned char   num_sattelites;      /*!< Number of sattelites */
		unsigned char   unk_4[24];
		double          ned_vel_c0;          /*!< NED velocity */
		double          ned_vel_c1;          /*!< NED velocity */
		double          ned_vel_c2;          /*!< NED velocity */
		double          pos_accur_c0;        /*!< Position accuracy */
		double          pos_accur_c1;        /*!< Position accuracy */
		double          pos_accur_c2;        /*!< Position accuracy */
		float           speed_acur;          /*!< Speed accuracy */
		float           time_acur;           /*!< Time accuracy */
		unsigned char   unk_5[72];
		float           temprature;
		float           pressure;
	} gps;

	// Check sum
	struct NAVDATA_CKS {
		unsigned short tag;
		unsigned short size;
		unsigned int   cks;
	} cks;
};

class ARDrone{
public:
	static const int AT_PORT = 5556;
	static const int NAVDATA_PORT = 5554;
	static const int VIDEO_PORT = 5555;
	static const int INTERVAL = 50;

	///IP and SOCKET
	char IP[20];

	UDPSocket sockCommand, sockNav;

	int Seqc;


	//Buffer 
	char In_Cmd[MAX_BUF_SIZE];

	//Nav 
	NAVDATA navdata;
	pthread_t *threadNav;
	pthread_mutex_t *mutexNav;
	static void *runNav(void *args) {
		reinterpret_cast<ARDrone*>(args)->loopNavdata();
		return NULL;
	}
	// Video 线程
	pthread_t *threadVideo, *threadAT;
	pthread_mutex_t *mutexVideo, *mutexCommand;
	static void *runAT(void *args) {
		reinterpret_cast<ARDrone*>(args)->loopAT();
		return NULL;
	}

	static void *runVideo(void *args) {
		reinterpret_cast<ARDrone*>(args)->loopVideo();
		return NULL;
	}

	// Camera image
	IplImage *img;
	// Video
	AVFormatContext *pFormatCtx;
	AVCodecContext  *pCodecCtx;
	AVFrame         *pFrame, *pFrameBGR;
	uint8_t         *bufferBGR;
	SwsContext      *pConvertCtx;
	bool            newImage;

	ARDrone::ARDrone(){
		strcpy_s(IP, "192.168.1.1");
		threadVideo = NULL;
		mutexVideo = NULL;
		threadNav = NULL;
		mutexNav = NULL;
		pFormatCtx = NULL;
		pCodecCtx = NULL;
		pFrame = pFrameBGR = NULL;
		bufferBGR = NULL;
		pConvertCtx = NULL;
		newImage = false;
		Seqc = 1;
	}
	//初始化
	bool Init(int Height_MAX = 3000, int VZ_MAX = 700){
		// Initialize FFmpeg
		av_register_all();
		avformat_network_init();
		av_log_set_level(AV_LOG_QUIET);
		if (!AT_init(Height_MAX, VZ_MAX)) return false;
		if (!Nav_init())return false;
		if (!Video_init())return false;
		return true;
	}
	int Video_init(){

		char filename[256];
		sprintf_s(filename, "tcp://%s:%d", IP, VIDEO_PORT);
		if (avformat_open_input(&pFormatCtx, filename, NULL, NULL) < 0) {
			cout << "avformat_open_input() was failed in Video Init" << endl;
			return 0;
		}
		else{
			cout << "avformat_open_input() OK" << endl;
		}

		// Retrive and dump stream information
		avformat_find_stream_info(pFormatCtx, NULL);
		av_dump_format(pFormatCtx, 0, filename, 0);

		// Find the decoder for the video stream
		pCodecCtx = pFormatCtx->streams[0]->codec;
		AVCodec *pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
		/*if (pCodec == avcodec_find_decoder(AV_CODEC_ID_H264)){
		cout << "Is H264" << endl;
		}*/
		if (pCodec == NULL) {
			cout << "avcodec_find_decoder() was failed in Video Init" << endl;
			return 0;
		}
		else{
			cout << "avcodec_find_decoder() OK" << endl;
		}
		// Open codec
		if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
			cout << "avcodec_open2() was failed.\n" << endl;
			return 0;
		}
		pFrame = av_frame_alloc();
		pFrameBGR = av_frame_alloc();
		bufferBGR = (uint8_t*)av_mallocz(avpicture_get_size(PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height) * sizeof(uint8_t));

		// Assign appropriate parts of buffer to image planes in pFrameBGR
		avpicture_fill((AVPicture*)pFrameBGR, bufferBGR, PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);

		// Convert it to BGR
		pConvertCtx = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height, PIX_FMT_BGR24, SWS_SPLINE, NULL, NULL, NULL);

		// Allocate an IplImage
		img = cvCreateImage(cvSize(pCodecCtx->width, (pCodecCtx->height == 368) ? 360 : pCodecCtx->height), IPL_DEPTH_8U, 3);
		if (!img) {
			cout << "cvCreateImage() was failed" << endl;
			return 0;
		}

		// Clear the image
		cvZero(img);

		// Create a mutex
		mutexVideo = new pthread_mutex_t;
		pthread_mutex_init(mutexVideo, NULL);

		// Create a thread
		threadVideo = new pthread_t;
		if (pthread_create(threadVideo, NULL, runVideo, this) != 0) {
			cout << "pthread_create() was failed." << endl;
			return 0;
		}
		else{
			cout << "pthread_Video OK" << endl;
		}

		return 1;
	}
	void loopVideo(){
		cout << "In video" << endl;
		while (true) {
			// Get video stream
			if (!getVideo()){
				break;
			}
			pthread_testcancel();
			msleep(2);
		}
	}
	bool getVideo(){
		AVPacket packet;
		int frameFinished = 0;
		// Read all frames
		while (av_read_frame(pFormatCtx, &packet) >= 0) {
			// Decode the frame
			avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, &packet);
			// Decoded all frames
			if (frameFinished) {
				// Convert to BGR
				if (mutexVideo) pthread_mutex_lock(mutexVideo);
				sws_scale(pConvertCtx, (const uint8_t* const*)pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrameBGR->data, pFrameBGR->linesize);
				newImage = true;
				if (mutexVideo) pthread_mutex_unlock(mutexVideo);

				// Free the packet and break immidiately
				av_free_packet(&packet);
				return true;
				//break;
			}

			// Free the packet
			av_free_packet(&packet);
		}
		return false;
	}
	cv::Mat getFrame(){
		if (!img) return cv::Mat();

		// Enable mutex lock
		if (mutexVideo) pthread_mutex_lock(mutexVideo);
		memcpy(img->imageData, pFrameBGR->data[0], pCodecCtx->width * ((pCodecCtx->height == 368) ? 360 : pCodecCtx->height) * sizeof(uint8_t) * 3);
		// The latest image has been read, so change newImage accordingly
		newImage = false;

		// Disable mutex lock
		if (mutexVideo) pthread_mutex_unlock(mutexVideo);
		return cv::cvarrToMat(img, true);
	}
	int AT_init(int Height_MAX=3000,int VZ_MAX=700){
		if (!sockCommand.open(IP, AT_PORT)) {
			cout << "UDPSocket::open AT OPRT Failed!\n" << endl;
			return 0;
		}
		else{
			cout << "AT Port OK!" << endl;
		}
		// Send undocumented command
		sockCommand.sendf("AT*PMODE=%d,%d\r", Get_Seq(), 2);

		// Send undocumented command
		sockCommand.sendf("AT*MISC=%d,%d,%d,%d,%d\r", Get_Seq(), 2, 20, 2000, 3000);

		// Send flat trim
		sockCommand.sendf("AT*FTRIM=%d,\r", Get_Seq());

		// Set the configuration IDs
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"custom:session_id\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID);
		msleep(500);
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"custom:profile_id\",\"%s\"\r", Get_Seq(), ARDRONE_PROFILE_ID);
		msleep(500);
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"custom:application_id\",\"%s\"\r", Get_Seq(), ARDRONE_APPLOCATION_ID);
		msleep(500);

		// Set maximum velocity in Z-axis [mm/s]
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"control:control_vz_max\",\"%d\"\r", Get_Seq(), VZ_MAX);
		msleep(100);

		// Set maximum yaw [rad/s]
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"control:control_yaw\",\"%f\"\r", Get_Seq(), 30.0 * DEG_TO_RAD);
		msleep(100);

		// Set maximum euler angle [rad]
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"control:euler_angle_max\",\"%f\"\r", Get_Seq(), 12.0 * DEG_TO_RAD);
		msleep(100);

		// Set maximum altitude [mm]
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"control:altitude_max\",\"%d\"\r", Get_Seq(), Height_MAX);
		msleep(100);

		// Bitrate control mode
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"%d\"\r", Get_Seq(), 0);     // VBC_MODE_DISABLED
		//sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"%d\"\r", Get_Seq(), 1);   // VBC_MODE_DYNAMIC
		//sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate_ctrl_mode\",\"%d\"\r", Get_Seq(), 2);   // VBC_MANUAL
		msleep(100);

		// Bitrate
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"video:bitrate\",\"%d\"\r", Get_Seq(), 1000);
		msleep(100);

		// Max bitrate
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"video:max_bitrate\",\"%d\"\r", Get_Seq(), 4000);
		msleep(100);

		// Set video codec
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", Get_Seq(), 0x81);   // H264_360P_CODEC
		//sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", Get_Seq(), 0x82); // MP4_360P_H264_720P_CODEC
		//sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", Get_Seq(), 0x83); // H264_720P_CODEC
		//sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"%d\"\r", Get_Seq(), 0x88); // MP4_360P_H264_360P_CODEC
		msleep(100);

		// Set video channel to default
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"video:video_channel\",\"0\"\r", Get_Seq());
		msleep(100);

		// Disable USB recording
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		sockCommand.sendf("AT*CONFIG=%d,\"video:video_on_usb\",\"FALSE\"\r", Get_Seq());
		msleep(100);

		// Disable outdoor mode
		//setOutdoorMode(false);

		// Create a mutex
		mutexCommand = new pthread_mutex_t;
		pthread_mutex_init(mutexCommand, NULL);

		// Create a thread
		threadAT = new pthread_t;
		if (pthread_create(threadAT, NULL, runAT, this) != 0) {
			cout << "pthread_create() was failed. AT!" << endl;
			return 0;
		}

		return 1;
	}
	void loopAT(){
		while (true) {
			// Reset Watch-Dog every 100ms
			if (mutexCommand) pthread_mutex_lock(mutexCommand);
			sockCommand.sendf("AT*COMWDG=%d\r", Get_Seq());
			if (mutexCommand) pthread_mutex_unlock(mutexCommand);
			pthread_testcancel();
			msleep(100);
		}
	}
	int Nav_init(){
		// Open the IP address and port
		if (!sockNav.open(IP, NAVDATA_PORT)) {
			cout << "UDPSocket::open !NAV! failed" << endl;
			return 0;
		}

		// Clear Navdata
		//memset(&navdata, 0, sizeof(navdata));

		// Start Navdata
		sockNav.sendf("\x01\x00\x00\x00");
		// Disable BOOTSTRAP mode
		if (mutexCommand) pthread_mutex_lock(mutexCommand);
		sockCommand.sendf("AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r", Get_Seq(), ARDRONE_SESSION_ID, ARDRONE_PROFILE_ID, ARDRONE_APPLOCATION_ID);
		//sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", Get_Seq());
		sockCommand.sendf("AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", Get_Seq());
		if (mutexCommand) pthread_mutex_unlock(mutexCommand);
		msleep(100);

		// Seed ACK
		sockCommand.sendf("AT*CTRL=%d,0\r", Get_Seq());
		// Create a mutex
		mutexNav = new pthread_mutex_t;
		pthread_mutex_init(mutexNav, NULL);

		// Create a thread
		threadNav = new pthread_t;
		if (pthread_create(threadNav, NULL, runNav, this) != 0) {
			cout << "pthread_create() NAV  failed" << endl;
			return 0;
		}
		else{
			cout << "Nav thread OK" << endl;
		}
		return 1;
	}
	void loopNavdata(){
		while (1) {
			// Get Navdata
			if (!getNavdata()) break;
			pthread_testcancel();
			msleep(30);
		}
	}
	bool getNavdata(){
		// Send a request
		sockNav.sendf("\x01\x00\x00\x00");
		// Receive data
		char buf[4096] = { '\0' };
		int size = sockNav.receive((void*)&buf, sizeof(buf));
		if (size > 0){
			////printf("Battery:  %d\n", GetIntFC(buf, NAVDATA_BATTERY));
			//cout << size << endl;
			// Enable mutex lock
			if (mutexNav) pthread_mutex_lock(mutexNav);

			// Header
			int index = 0;
			memcpy((void*)&(navdata.header), (const void*)(buf + index), 4); index += 4;
			memcpy((void*)&(navdata.ardrone_state), (const void*)(buf + index), 4); index += 4;
			memcpy((void*)&(navdata.sequence), (const void*)(buf + index), 4); index += 4;
			memcpy((void*)&(navdata.vision_defined), (const void*)(buf + index), 4); index += 4;
			// Parse navdata
			while (index < size) {
				// Tag and data size
				unsigned short tmp_tag, tmp_size;
				memcpy((void*)&tmp_tag, (const void*)(buf + index), 2); index += 2;  // tag
				memcpy((void*)&tmp_size, (const void*)(buf + index), 2); index += 2;  // size
				index -= 4;
				// Copy to NAVDATA structure
				switch (tmp_tag) {
				case ARDRONE_NAVDATA_DEMO_TAG:
					memcpy((void*)&(navdata.demo), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.demo)));
					break;
				case ARDRONE_NAVDATA_TIME_TAG:
					memcpy((void*)&(navdata.time), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.time)));
					break;
				case ARDRONE_NAVDATA_RAW_MEASURES_TAG:
					memcpy((void*)&(navdata.raw_measures), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.raw_measures)));
					break;
				case ARDRONE_NAVDATA_PHYS_MEASURES_TAG:
					memcpy((void*)&(navdata.phys_measures), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.phys_measures)));
					break;
				case ARDRONE_NAVDATA_GYROS_OFFSETS_TAG:
					memcpy((void*)&(navdata.gyros_offsets), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.gyros_offsets)));
					break;
				case ARDRONE_NAVDATA_EULER_ANGLES_TAG:
					memcpy((void*)&(navdata.euler_angles), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.euler_angles)));
					break;
				case ARDRONE_NAVDATA_REFERENCES_TAG:
					memcpy((void*)&(navdata.references), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.references)));
					break;
				case ARDRONE_NAVDATA_TRIMS_TAG:
					memcpy((void*)&(navdata.trims), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.trims)));
					break;
				case ARDRONE_NAVDATA_RC_REFERENCES_TAG:
					memcpy((void*)&(navdata.rc_references), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.rc_references)));
					break;
				case ARDRONE_NAVDATA_PWM_TAG:
					memcpy((void*)&(navdata.pwm), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.pwm)));
					break;
				case ARDRONE_NAVDATA_ALTITUDE_TAG:
					memcpy((void*)&(navdata.altitude), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.altitude)));
					break;
				case ARDRONE_NAVDATA_VISION_RAW_TAG:
					memcpy((void*)&(navdata.vision_raw), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision_raw)));
					break;
				case ARDRONE_NAVDATA_VISION_OF_TAG:
					memcpy((void*)&(navdata.vision_of), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision_of)));
					break;
				case ARDRONE_NAVDATA_VISION_TAG:
					memcpy((void*)&(navdata.vision), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision)));
					break;
				case ARDRONE_NAVDATA_VISION_PERF_TAG:
					memcpy((void*)&(navdata.vision_perf), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision_perf)));
					break;
				case ARDRONE_NAVDATA_TRACKERS_SEND_TAG:
					memcpy((void*)&(navdata.trackers_send), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.trackers_send)));
					break;
				case ARDRONE_NAVDATA_VISION_DETECT_TAG:
					memcpy((void*)&(navdata.vision_detect), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.vision_detect)));
					break;
				case ARDRONE_NAVDATA_WATCHDOG_TAG:
					memcpy((void*)&(navdata.watchdog), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.watchdog)));
					break;
				case ARDRONE_NAVDATA_ADC_DATA_FRAME_TAG:
					memcpy((void*)&(navdata.adc_data_frame), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.adc_data_frame)));
					break;
				case ARDRONE_NAVDATA_VIDEO_STREAM_TAG:
					memcpy((void*)&(navdata.video_stream), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.video_stream)));
					break;
				case ARDRONE_NAVDATA_GAME_TAG:
					memcpy((void*)&(navdata.games), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.games)));
					break;
				case ARDRONE_NAVDATA_PRESSURE_RAW_TAG:
					memcpy((void*)&(navdata.pressure_raw), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.pressure_raw)));
					break;
				case ARDRONE_NAVDATA_MAGNETO_TAG:
					memcpy((void*)&(navdata.magneto), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.magneto)));
					break;
				case ARDRONE_NAVDATA_WIND_TAG:
					memcpy((void*)&(navdata.wind), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.wind)));
					break;
				case ARDRONE_NAVDATA_KALMAN_PRESSURE_TAG:
					memcpy((void*)&(navdata.kalman_pressure), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.kalman_pressure)));
					break;
				case ARDRONE_NAVDATA_HDVIDEO_STREAM_TAG:
					memcpy((void*)&(navdata.hdvideo_stream), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.hdvideo_stream)));
					break;
				case ARDRONE_NAVDATA_WIFI_TAG:
					memcpy((void*)&(navdata.wifi), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.wifi)));
					break;
				case ARDRONE_NAVDATA_GPS_TAG:
					// Unuse GPS
					/*
					if (version.major == 2 && version.minor == 4) memcpy((void*)&(navdata.gps), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.gps)));
					else                                          memcpy((void*)&(navdata.zimmu_3000), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.zimmu_3000)));
					*/
					break;
				case 28:
					break;
				case 29:
					break;
				default:
					//cout << "TAG:" << tmp_tag << endl;
					//cout << "SIZE:" << tmp_size << endl;
					memcpy((void*)&(navdata.cks), (const void*)(buf + index), MIN(tmp_size, sizeof(navdata.cks)));
					break;
				}
				index += tmp_size;
			}
			// Disable mutex lock
			if (mutexNav) pthread_mutex_unlock(mutexNav);
		}
		return true;
	}
	

	// Command

	//! Disable the emergency lock.
	void ARDrone::resetEmergency(void)
	{
		// Get the state
		if (mutexCommand) pthread_mutex_lock(mutexNav);
		int state = navdata.ardrone_state;
		if (mutexCommand) pthread_mutex_unlock(mutexNav);

		// If AR.Drone is in emergency, reset it
		if (state & ARDRONE_EMERGENCY_MASK) {
			if (mutexCommand) pthread_mutex_lock(mutexCommand);
			sockCommand.sendf("AT*REF=%d,290717952\r", Get_Seq());
			if (mutexCommand) pthread_mutex_unlock(mutexCommand);
		}
	}

	//   Calibrate AR.Drone's magnetometer.
	void ARDrone::setCalibration(int device = 0)
	{
		if (!onGround()) {
			// Send calibration command
			if (mutexCommand) pthread_mutex_lock(mutexCommand);
			sockCommand.sendf("AT*CALIB=%d,%d\r", Get_Seq(), device);
			if (mutexCommand) pthread_mutex_unlock(mutexCommand);
		}
	}

	// --------------------------------------------------------------------------
	//! @brief   Move the AR.Drone in 2D plane.
	//! @param   vx X velocity [m/s]
	//! @param   vy Y velocity [m/s]
	//! @param   vr Angular velocity [rad/s]
	//! @return  None
	// --------------------------------------------------------------------------
	void move(double vx, double vy, double vr){
		move3D(vx, vy, 0.0, vr);
	}
	// --------------------------------------------------------------------------
	//! @brief   Move the AR.Drone in 3D space.
	//! @param   vx X velocity [m/s]
	//! @param   vy Y velocity [m/s]
	//! @param   vz Z velocity [m/s]
	//! @param   vr Angular velocity [rad/s]
	//! @return  None
	// --------------------------------------------------------------------------
	void ARDrone::move3D(double vx, double vy, double vz, double vr)
	{
		// AR.Drone is flying
		if (!onGround()) {
			// Command velocities
			float v[4] = { -0.2f * (float)vy, -0.2f * (float)vx, 1.0f * (float)vz, -0.5f * (float)vr };
			int mode = (fabs(v[0]) > 0.0 || fabs(v[1]) > 0.0);

			// Nomarization (-1.0 to +1.0)
			for (int i = 0; i < 4; i++) {
				if (fabs(v[i]) > 1.0) v[i] /= fabs(v[i]);
			}

			// Send a command
			if (mutexCommand) pthread_mutex_lock(mutexCommand);
			sockCommand.sendf("AT*PCMD=%d,%d,%d,%d,%d,%d\r", Get_Seq(), mode, *(int*)(&v[0]), *(int*)(&v[1]), *(int*)(&v[2]), *(int*)(&v[3]));
			if (mutexCommand) pthread_mutex_unlock(mutexCommand);
		}
	}
	void ARDrone::move3D(SPEED speed){
		move3D(speed.vx, speed.vy, speed.vz, speed.vr);
	}
	
	// Set a reference of the horizontal plane.
	void ARDrone::setFlatTrim(void)
	{
		if (onGround()) {
			// Send flat trim command
			if (mutexCommand) pthread_mutex_lock(mutexCommand);
			sockCommand.sendf("AT*FTRIM=%d\r", Get_Seq());
			if (mutexCommand) pthread_mutex_unlock(mutexCommand);
		}
	}
	void emergency(void)
	{
		// Send emergency
		if (mutexCommand) pthread_mutex_lock(mutexCommand);
		sockCommand.sendf("AT*REF=%d,290717952\r", Get_Seq());
		if (mutexCommand) pthread_mutex_unlock(mutexCommand);
	}
	void takeoff(void){
		// Get the state
		if (mutexCommand) pthread_mutex_lock(mutexNav);
		int state = navdata.ardrone_state;
		if (mutexCommand) pthread_mutex_unlock(mutexNav);

		// If AR.Drone is in emergency, reset it
		if (state & ARDRONE_EMERGENCY_MASK) emergency();
		else {
			// Send take off
			if (mutexCommand) pthread_mutex_lock(mutexCommand);
			sockCommand.sendf("AT*REF=%d,290718208\r", Get_Seq());
			if (mutexCommand) pthread_mutex_unlock(mutexCommand);
		}
	}
	void ARDrone::landing(void){
		// Send langding
		if (mutexCommand) pthread_mutex_lock(mutexCommand);
		sockCommand.sendf("AT*REF=%d,290717696\r", Get_Seq());
		if (mutexCommand) pthread_mutex_unlock(mutexCommand);

	}

	// 传感器
	int ARDrone::onGround(void)
	{
		// Get the data
		if (mutexNav) pthread_mutex_lock(mutexNav);
		int on_ground = (navdata.ardrone_state & ARDRONE_FLY_MASK) ? 0 : 1;
		if (mutexNav) pthread_mutex_unlock(mutexNav);

		return on_ground;
	}
	// --------------------------------------------------------------------------
	//! @brief   Get current role angle of AR.Drone.
	//! @return  Role angle [rad]
	// --------------------------------------------------------------------------
	double ARDrone::getRoll(void)
	{
		// Get the data
		if (mutexNav) pthread_mutex_lock(mutexNav);
		double roll = navdata.demo.phi * 0.001 * DEG_TO_RAD;
		if (mutexNav) pthread_mutex_unlock(mutexNav);

		return roll;
	}

	// --------------------------------------------------------------------------
	//! @brief   Get current pitch angle of AR.Drone.
	//! @return  Pitch angle [rad]
	// --------------------------------------------------------------------------
	double ARDrone::getPitch(void)
	{
		// Get the data
		if (mutexNav) pthread_mutex_lock(mutexNav);
		double pitch = -navdata.demo.theta * 0.001 * DEG_TO_RAD;
		if (mutexNav) pthread_mutex_unlock(mutexNav);

		return pitch;
	}

	// --------------------------------------------------------------------------
	//! @brief   Get current yaw angle of AR.Drone.
	//! @return  Yaw angle [rad]
	// --------------------------------------------------------------------------
	double ARDrone::getYaw(void)
	{
		// Get the data
		if (mutexNav) pthread_mutex_lock(mutexNav);
		double yaw = -navdata.demo.psi * 0.001 * DEG_TO_RAD;
		if (mutexNav) pthread_mutex_unlock(mutexNav);

		return yaw;
	}

	// --------------------------------------------------------------------------
	//! @brief   Get current altitude of AR.Drone.
	//! @return  Altitude [m]
	// --------------------------------------------------------------------------
	double ARDrone::getAltitude(void)
	{
		// Get the data
		if (mutexNav) pthread_mutex_lock(mutexNav);
		double altitude = navdata.demo.altitude * 0.001;
		if (mutexNav) pthread_mutex_unlock(mutexNav);

		return altitude;
	}

	// --------------------------------------------------------------------------
	//! @brief   Get estimated velocity of AR.Drone.
	//! @param   vx A pointer to the X velocity variable [m/s]
	//! @param   vy A pointer to the Y velocity variable [m/s]
	//! @param   vz A pointer to the Z velocity variable [m/s]
	//! @return Velocity [m/s]
	// --------------------------------------------------------------------------
	double getVelocity(double *vx, double *vy, double *vz)
	{
		// Get the data
		if (mutexNav) pthread_mutex_lock(mutexNav);
		double velocity_x = navdata.demo.vx * 0.001;
		double velocity_y = -navdata.demo.vy * 0.001;
		//double velocity_z = -navdata.demo.vz * 0.001;
		double velocity_z = -navdata.altitude.altitude_vz * 0.001;
		if (mutexNav) pthread_mutex_unlock(mutexNav);

		// Velocities
		if (vx) *vx = velocity_x;
		if (vy) *vy = velocity_y;
		if (vz) *vz = velocity_z;

		// Velocity [m/s]
		double velocity = sqrt(velocity_x*velocity_x + velocity_y*velocity_y + velocity_z*velocity_z);
		return velocity;
	}

	int ARDrone::getBatteryPercentage(void)
	{
		// Get the data
		if (mutexNav) pthread_mutex_lock(mutexNav);
		int battery = navdata.demo.vbat_flying_percentage;
		if (mutexNav) pthread_mutex_unlock(mutexNav);

		return battery;
	}
	int ARDrone::getWIFI_S(void)
	{
		// Get the data
		if (mutexNav) pthread_mutex_lock(mutexNav);
		int wifi = navdata.wifi.link_quality;
		if (mutexNav) pthread_mutex_unlock(mutexNav);

		return wifi;
	}

	int Get_Seq(){
		return Seqc++;
	}
	void finalizeVideo(void){
		// Destroy the thread
		if (threadVideo) {
			pthread_cancel(*threadVideo);
			pthread_join(*threadVideo, NULL);
			delete threadVideo;
			threadVideo = NULL;
		}
		// Delete the mutex
		if (mutexVideo) {
			pthread_mutex_destroy(mutexVideo);
			delete mutexVideo;
			mutexVideo = NULL;
		}
		// Release the IplImage
		if (img) {
			cvReleaseImage(&img);
			img = NULL;
		}
		// Deallocate the frame
		if (pFrame) {
			av_frame_free(&pFrame);
			pFrame = NULL;
		}
		// Deallocate the frame
		if (pFrameBGR) {
			av_frame_free(&pFrameBGR);
			pFrameBGR = NULL;
		}
		// Deallocate the buffer
		if (bufferBGR) {
			av_free(bufferBGR);
			bufferBGR = NULL;
		}
		// Deallocate the convert context
		if (pConvertCtx) {
			sws_freeContext(pConvertCtx);
			pConvertCtx = NULL;
		}
		// Deallocate the codec
		if (pCodecCtx) {
			avcodec_close(pCodecCtx);
			pCodecCtx = NULL;
		}
		// Deallocate the format context
		if (pFormatCtx) {
			avformat_close_input(&pFormatCtx);
			pFormatCtx = NULL;
		}
	}
	void finalizeNavdata(void){
		// Destroy the thread
		if (threadNav) {
			pthread_cancel(*threadNav);
			pthread_join(*threadNav, NULL);
			delete threadNav;
			threadNav = NULL;
		}

		// Delete the mutex
		if (mutexNav) {
			pthread_mutex_destroy(mutexNav);
			delete mutexNav;
			mutexNav = NULL;
		}

		// Close the socket
		sockNav.close();
	}
	void finalizeCommand(void)
	{
		// Destroy the thread
		if (threadAT) {
			pthread_cancel(*threadAT);
			pthread_join(*threadAT, NULL);
			delete threadAT;
			threadAT = NULL;
		}

		// Delete the mutex
		if (mutexCommand) {
			pthread_mutex_destroy(mutexCommand);
			delete mutexCommand;
			mutexCommand = NULL;
		}

		// Close the socket
		sockCommand.close();
	}
	void Close(){
		finalizeVideo();
		finalizeNavdata();
		finalizeCommand();
	}
};


