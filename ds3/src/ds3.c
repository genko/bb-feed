// linmctool - Command-line tool for motion-sensing Bluetooth controllers.
// Copyright (c) 2010,2011 pabr@pabr.org
// See http://www.pabr.org/linmctool/
//
// Compile with: gcc --std=gnu99 -Wall linmctool.c -lusb -o linmctool

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>

// Global configuration

int verbose = 0;
int enable_ir = 0;
int enable_wmp = 0;
int def_rgb[3] = { 0, 0, 0 };
int output_text = 0;
int repeat_dump = 1;
int poll_report = -1;
char *usb_force_master = NULL;
int force_ds3 = 0;
int timestamp = 0;
int nostdin = 0;

struct ds3_st *ds3;

void fatal(char *msg) {
  if ( errno ) perror(msg); else fprintf(stderr, "%s\n", msg);
  exit(1);
}

// ----------------------------------------------------------------------
// Replacement for libbluetooth

int mystr2ba(const char *s, bdaddr_t *ba) {
  if ( strlen(s) != 17 ) return 1;
  for ( int i=0; i<6; ++i ) {
    int d = strtol(s+15-3*i, NULL, 16);
    if ( d<0 || d>255 ) return 1;
    ba->b[i] = d;
  }
  return 0;
}

const char *myba2str(const bdaddr_t *ba) {
  static char buf[2][18];  // Static buffer valid for two invocations.
  static int index = 0;
  index = (index+1)%2;
  sprintf(buf[index], "%02x:%02x:%02x:%02x:%02x:%02x",
	  ba->b[5], ba->b[4], ba->b[3], ba->b[2], ba->b[1], ba->b[0]);
  return buf[index];
}

static const char *rtypename[] = { "OTHER", "INPUT", "OUTPUT", "FEATURE" };

// ----------------------------------------------------------------------
// USB support

#ifndef WITHOUT_USB

#include <usb.h>
#define USB_DIR_IN 0x80
#define USB_DIR_OUT 0
#define USB_GET_REPORT 0x01
#define USB_SET_REPORT 0x09
#define VENDOR_SONY 0x054c
#define PRODUCT_SIXAXIS_DS3 0x0268

void usb_dump_readable_reports(usb_dev_handle *devh, int itfnum) {
  for ( int rtype=0; rtype<=3; ++rtype )
    for ( int rid=0; rid<256; ++rid ) {
      unsigned char prev[64];
      for ( int c=0; c<repeat_dump; ++c ) {
	unsigned char r[64];
	int nr = usb_control_msg
	  (devh, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	   USB_GET_REPORT, (rtype<<8)|rid, itfnum, (void*)r, sizeof(r), 5000);
	if ( c==0 || nr<0 || memcmp(r,prev,nr) ) {
	  printf("  USB READ %-7s 0x%02x %c ",
		 rtypename[rtype], rid, c?'#':'=');
	  if ( nr < 0 )
	    printf("ERR %d", nr);
	  else {
	    printf("[%3d]", nr);
	    for ( int i=0; i<nr; ++i ) printf(" %02x", r[i]);
	  }
	  printf("\n");  fflush(stdout);
	}
	memcpy(prev, r, sizeof(prev));
      }
    }
  exit(0);
}

void usb_dump_writable_reports(usb_dev_handle *devh, int itfnum) {
  for ( int rtype=0; rtype<=3; ++rtype )
    for ( int rid=0; rid<256; ++rid ) {
      unsigned char r[16];
      memset(r, 0, sizeof(r));
      int nr = usb_control_msg
	(devh, USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	 USB_SET_REPORT, (rtype<<8)|rid, itfnum, (void*)r, sizeof(r), 5000);
      printf("  USB WRITE %-7s 0x%02x [%d] -> [%d] %s\n",
	      rtypename[rtype], rid, sizeof(r), nr, (nr<0)?strerror(-nr):"");
      fflush(stdout);
    }
  exit(0);
}

void usb_poll_report(usb_dev_handle *devh, int itfnum, int report) {
  int rtype = report >> 8;
  int rid = report & 255;
  while ( 1 ) {
    unsigned char r[64];
    int nr = usb_control_msg
      (devh, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
       USB_GET_REPORT, (rtype<<8)|rid, itfnum, (void*)r, sizeof(r), 5000);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    printf("  %10lu.%06ld USB READ %-7s 0x%02x ",
	   tv.tv_sec, tv.tv_usec, rtypename[rtype], rid);
    if ( nr < 0 )
      printf("ERR %d", nr);
    else {
      printf("[%3d]", nr);
      for ( int i=0; i<nr; ++i ) printf(" %02x", r[i]);
    }
    printf("\n");
  }
}

void usb_pair_device(struct usb_device *dev, int itfnum) {

  usb_dev_handle *devh = usb_open(dev);
  if ( ! devh ) fatal("usb_open");
  usb_detach_kernel_driver_np(devh, itfnum);
  int res = usb_claim_interface(devh, itfnum);
  if ( res < 0 ) fatal("usb_claim_interface");

  if ( poll_report >= 0 ) usb_poll_report(devh, itfnum, poll_report);

  bdaddr_t current_ba;  // Current pairing address.

  switch ( dev->descriptor.idProduct ) {
  case PRODUCT_SIXAXIS_DS3: {
    fprintf(stderr, "USB: SIXAXIS/DS3\n");
    unsigned char msg[8];
    res = usb_control_msg
      (devh, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
       USB_GET_REPORT, 0x03f5, itfnum, (void*)msg, sizeof(msg), 5000);
    if ( res < 0 ) fatal("usb_control_msg(read master)");
    for ( int i=0; i<6; ++i ) current_ba.b[i] = msg[7-i];
    break;
  }
  }

  bdaddr_t ba;  // New pairing address.

  if ( usb_force_master && !mystr2ba(usb_force_master,&ba) )
    ;
  else {
    char ba_s[18];
    FILE *f = popen("hcitool dev", "r");
    if ( !f || fscanf(f, "%*s\n%*s %17s", ba_s)!=1 || mystr2ba(ba_s, &ba) )
      fatal("Unable to retrieve local bd_addr from `hcitool dev`.\n");
    pclose(f);
  }

  // Perform pairing.

  if ( ! bacmp(&current_ba, &ba) ) {
    fprintf(stderr, "  Already paired to %s\n", myba2str(&ba));
  } else {
    fprintf(stderr, "  Changing master from %s to %s\n",
	    myba2str(&current_ba), myba2str(&ba));
    switch ( dev->descriptor.idProduct ) {
    case PRODUCT_SIXAXIS_DS3: {
      char msg[8] =
	{ 0x01, 0x00, ba.b[5],ba.b[4],ba.b[3],ba.b[2],ba.b[1],ba.b[0] };
      res = usb_control_msg
	(devh, USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	 USB_SET_REPORT, 0x03f5, itfnum, msg, sizeof(msg), 5000);
      if ( res < 0 ) fatal("usb_control_msg(write master)");
      break;
    }
    }
  }

  if ( dev->descriptor.idProduct == PRODUCT_SIXAXIS_DS3 )
    fprintf(stderr, "  Now unplug the USB cable and press the PS button.\n");
  else
    fprintf(stderr, "  Now press the PS button.\n");
}

void usb_scan() {
  usb_init();
  if ( usb_find_busses() < 0 ) fatal("usb_find_busses");
  if ( usb_find_devices() < 0 ) fatal("usb_find_devices");
  struct usb_bus *busses = usb_get_busses();
  if ( ! busses ) fatal("usb_get_busses");

  struct usb_bus *bus;
  for ( bus=busses; bus; bus=bus->next ) {
    struct usb_device *dev;
    for ( dev=bus->devices; dev; dev=dev->next) {
      struct usb_config_descriptor *cfg;
      for ( cfg = dev->config;
	    cfg < dev->config + dev->descriptor.bNumConfigurations;
	    ++cfg ) {
	int itfnum;
	for ( itfnum=0; itfnum<cfg->bNumInterfaces; ++itfnum ) {
	  struct usb_interface *itf = &cfg->interface[itfnum];
	  struct usb_interface_descriptor *alt;
	  for ( alt = itf->altsetting;
		alt < itf->altsetting + itf->num_altsetting;
		++alt ) {
	    if ( dev->descriptor.idVendor == VENDOR_SONY &&
		 (dev->descriptor.idProduct == PRODUCT_SIXAXIS_DS3) &&
		 alt->bInterfaceClass == 3 )
	      usb_pair_device(dev, itfnum);
	  }
	}
      }
    }
  }
}

#else // WITHOUT_USB

void usb_scan() { }

#endif

/**********************************************************************/
// Bluetooth HID devices

static const char *devtypename[] =
  { "DS3" };

struct motion_dev {
  int index;
  bdaddr_t addr;
  enum { DS3 } type;
  int csk;
  int isk;
  struct motion_dev *next;
};

struct ds3_st {
  ushort sel; 
  ushort start;
  ushort dup; ushort dup_p;
  ushort ddown; ushort ddown_p;
  ushort dleft; ushort dleft_p;
  ushort dright; ushort dright_p;
  ushort circ; ushort circ_p;
  ushort square; ushort square_p;
  ushort tri; ushort tri_p;
  ushort ex; ushort ex_p;
  ushort l1; ushort l1_p;
  ushort l2; ushort l2_p;
  ushort r1; ushort r1_p;
  ushort r2; ushort r2_p;
  ushort joylx; ushort joyly;
  ushort joyrx; ushort joyry;
  ushort charge;
};

/* Bluetooth L2CAP PSM */
#define L2CAP_PSM_HIDP_CTRL 0x11
#define L2CAP_PSM_HIDP_INTR 0x13

#define HIDP_TRANS_GET_REPORT    0x40
#define HIDP_TRANS_SET_REPORT    0x50
#define HIDP_DATA_RTYPE_INPUT    0x01
#define HIDP_DATA_RTYPE_OUTPUT   0x02
#define HIDP_DATA_RTYPE_FEATURE  0x03

void bluetooth_dump_readable_reports(int csk) {
  for ( int rtype=0; rtype<=3; ++rtype )
    for ( int rid=0; rid<256; ++rid ) {
      unsigned char prev[64];
      for ( int c=0; c<repeat_dump; ++c ) {
	unsigned char r[66];
	char get[] = { HIDP_TRANS_GET_REPORT | rtype | 8,
		       rid, sizeof(r), sizeof(r)>>8 };
	send(csk, get, sizeof(get), 0);
	int nr = recv(csk, r, sizeof(r), 0);
	if ( c==0 || nr<0 || memcmp(r,prev,nr) ) {
	  printf("  BLUETOOTH READ %-7s 0x%02x %c ",
		 rtypename[rtype], rid, c?'#':'=');
	  if ( nr < 1 ) printf("ERR %d", nr);
	  else if ( r[0] == (0xa0|rtype) )
	    printf("[%3d]", nr-1);
	  else
	    printf("ERR %d", r[0]);
	  for ( int i=1; i<nr; ++i ) printf(" %02x", r[i]);
	  printf("\n"); fflush(stdout);
	}
	memcpy(prev, r, sizeof(prev));
      }
    }
  exit(0);
}

void bluetooth_dump_writable_reports(int csk) {
  for ( int rtype=0; rtype<=3; ++rtype )
    for ( int rid=0; rid<256; ++rid ) {
      char set[] = { HIDP_TRANS_SET_REPORT | rtype, rid,
		     0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
      send(csk, set, sizeof(set), 0);
      unsigned char ack[16];
      int nr = recv(csk, ack, sizeof(ack), 0);
      printf("  BLUETOOTH WRITE %-7s 0x%02x [%d] -> [%d] %02x\n",
	      rtypename[rtype], rid, sizeof(set)-2, nr, ack[0]);
      fflush(stdout);
    }
  exit(0);
}

void bluetooth_poll_report(int csk, int report) {
  int rtype = report >> 8;
  int rid = report & 255;
  while ( 1 ) {
    unsigned char r[66];
    char get[] = { HIDP_TRANS_GET_REPORT | rtype | 8,
		   rid, sizeof(r), sizeof(r)>>8 };
    send(csk, get, sizeof(get), 0);
    int nr = recv(csk, r, sizeof(r), 0);
    if ( nr < 0 ) fatal("recv");
    struct timeval tv;
    gettimeofday(&tv, NULL);
    printf("  %10lu.%06ld BLUETOOTH READ %-7s 0x%02x ",
	   tv.tv_sec, tv.tv_usec, rtypename[rtype], rid);
    if ( r[0] == (0xa0|rtype) )
      printf("[%3d]", nr-1);
    else
      printf("ERR %d", r[0]);
    for ( int i=1; i<nr; ++i ) printf(" %02x", r[i]);
    printf("\n");
  }
}

// Incoming connections.

int l2cap_listen(const bdaddr_t *bdaddr, unsigned short psm) {
  int sk = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
  if ( sk < 0 ) fatal("socket");

  struct sockaddr_l2 addr;
  memset(&addr, 0, sizeof(addr));
  addr.l2_family = AF_BLUETOOTH;
  addr.l2_bdaddr = *BDADDR_ANY;
  addr.l2_psm = htobs(psm);
  if ( bind(sk, (struct sockaddr *)&addr, sizeof(addr)) < 0 ) {
    perror("bind");
    close(sk);
    return -1;
  }

  if ( listen(sk, 5) < 0 ) fatal("listen");
  return sk;
}

struct motion_dev *accept_device(int csk, int isk) {
  fprintf(stderr, "Incoming connection...\n");
  struct motion_dev *dev = malloc(sizeof(struct motion_dev));
  if ( ! dev ) fatal("malloc");

  dev->csk = accept(csk, NULL, NULL);
  if ( dev->csk < 0 ) fatal("accept(CTRL)");
  dev->isk = accept(isk, NULL, NULL);
  if ( dev->isk < 0 ) fatal("accept(INTR)");

  struct sockaddr_l2 addr;
  socklen_t addrlen = sizeof(addr);
  if ( getpeername(dev->isk, (struct sockaddr *)&addr, &addrlen) < 0 )
    fatal("getpeername");
  dev->addr = addr.l2_bdaddr;

  {
    // DS3.
    unsigned char resp[64];
    char get03f2[] = { HIDP_TRANS_GET_REPORT | HIDP_DATA_RTYPE_FEATURE | 8,
		       0xf2, sizeof(resp), sizeof(resp)>>8 };
    send(dev->csk, get03f2, sizeof(get03f2), 0);  // 0301 is interesting too.
    int nr = recv(dev->csk, resp, sizeof(resp), 0);
    if (( nr>= 19 ) || ( force_ds3 )) dev->type = DS3;
    else dev->type = DS3;  // My guess.
  }

  return dev;
}

// Outgoing connections.

int l2cap_connect(bdaddr_t *ba, unsigned short psm) {
  int sk = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
  if ( sk < 0 ) fatal("socket");

  struct sockaddr_l2 daddr;
  memset(&daddr, 0, sizeof(daddr));
  daddr.l2_family = AF_BLUETOOTH;
  daddr.l2_bdaddr = *ba;
  daddr.l2_psm = htobs(psm);
  if ( connect(sk, (struct sockaddr *)&daddr, sizeof(daddr)) < 0 )
    fatal("connect");

  return sk;
}

/**********************************************************************/
// Device setup

#define IR0 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00, 0x41
#define IR1 0x40, 0x00

#define BIT1 2

void dump(const char *tag, const unsigned char *buf, int len) {
  fprintf(stderr, "%s[%d]", tag, len);
  for ( int i=0; i<len; ++i ) fprintf(stderr, " %02x", buf[i]);
  fprintf(stderr, "\n");
}

void hidp_trans(int csk, const char *buf, int len) {
  if ( verbose ) dump("SEND", (unsigned char*)buf, len);
  if ( send(csk, buf, len, 0) != len ) fatal("send(CTRL)");
  unsigned char ack;
  int nr = recv(csk, &ack, sizeof(ack), 0);
  if ( verbose) fprintf(stderr, "    result %d  %02x\n", nr, ack);
  if ( nr!=1 || ack!=0 ) fatal("ack");
}

void setup_device(struct motion_dev *dev) {
  if ( poll_report >= 0 ) bluetooth_poll_report(dev->csk, poll_report);

  switch ( dev->type ) {
  case DS3: {
    // Enable reporting
    char set03f4[] = { HIDP_TRANS_SET_REPORT | HIDP_DATA_RTYPE_FEATURE, 0xf4,
		       0x42, 0x03, 0x00, 0x00 };
    hidp_trans(dev->csk, set03f4, sizeof(set03f4));
    // Leds: Display 1+index in additive format.
    static const char ledmask[10] = { 1, 2, 4, 8, 6, 7, 11, 13, 14, 15 };
    #define LED_PERMANENT 0xff, 0x27, 0x00, 0x00, 0x32
    char set0201[] = {
      HIDP_TRANS_SET_REPORT | HIDP_DATA_RTYPE_OUTPUT, 0x01,
      0x00, 0x00, 0x00, 0,0, 
      0x00, 0x00, 0x00,
      0x00, ledmask[dev->index%10]<<1,
      LED_PERMANENT,
      LED_PERMANENT,
      LED_PERMANENT,
      LED_PERMANENT,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    //right rumble
    set0201[3] = 0x40; // rumble time
    set0201[4] = 0xff; // rubmle intens
    //left rumble
    set0201[5] = 0x40;     // Set to e.g. 20 to test rumble on startup
    set0201[6] = 0xff;  // Weak rumble
    hidp_trans(dev->csk, set0201, sizeof(set0201));
    break;
  }
  }
  fprintf(stderr, "New device %d %s is a %s\n",
	  dev->index, myba2str(&dev->addr), devtypename[dev->type]);
}
/**********************************************************************/
// Reports
void sixaxis_ds3_parse_report(unsigned char *r, int len) {
  ds3->sel = r[2]&0x01;
  ds3->start = (r[2]&0x08)>>3;
  ds3->tri_p = r[22]; //24
  ds3->square_p = r[25];
  ds3->ex_p = r[24];
  ds3->circ_p = r[23];
  ds3->ddown_p = r[16];
  ds3->dup_p = r[14]; //16
  ds3->dleft_p = r[17];
  ds3->dright_p = r[15];
  ds3->l1_p = r[20];
  ds3->l2_p = r[18];
  ds3->r1_p = r[21];
  ds3->r2_p = r[19];
  ds3->joylx = r[6];
  ds3->joyly = r[7];
  ds3->joyrx = r[8];
  ds3->joyry = r[9];
  ds3->charge = r[30];
  printf("%i %i %2x:%2x ",ds3->sel,ds3->start,ds3->joylx,ds3->joyly);
  printf("%2x:%2x ",ds3->joyrx,ds3->joyry);
  printf("%2x ",ds3->dup_p);
  printf("%2x ",ds3->ddown_p);
  printf("%2x ",ds3->dleft_p);
  printf("%2x ",ds3->dright_p);
  printf("%2x ",ds3->circ_p); 
  printf("%2x ",ds3->square_p); 
  printf("%2x ",ds3->tri_p);  
  printf("%2x ",ds3->ex_p);  
  printf("%2x ",ds3->l1_p); 
  printf("%2x ",ds3->l2_p);
  printf("%2x ",ds3->r1_p); 
  printf("%2x ",ds3->r2_p);
  if ( r[0]==0x01 && len>=49 ) {
    int aX = r[41]*256 + r[42] - 512;
    int aY = r[43]*256 + r[44] - 512;
    int aZ = r[45]*256 + r[46] - 512;
    int gZ = r[47]*256 + r[48] - 512;
    printf(" aX=%-4d aY=%-4d aZ=%-4d gZ=%-4d", aX,aY,aZ, gZ);
  }
  printf(" Chrg:%2x \n",ds3->charge);
}

void parse_report(struct motion_dev *dev, unsigned char *r, int len) {
  struct timeval tv;
  if ( timestamp ) {
    gettimeofday(&tv, NULL);
    printf("%10lu.%06ld ", tv.tv_sec, tv.tv_usec);
  }
  switch ( dev->type ) {
  case DS3:
    printf("%d %s DS3 ", dev->index, myba2str(&dev->addr));
    sixaxis_ds3_parse_report(r, len);
    break;
  }
  fflush(stdout);
}

/**********************************************************************/
// Main

void usage() {
  fprintf(stderr, "linmctool version TBD.\n");
  fprintf(stderr, "Usage: linmctool [options] [BDADDR...]\n"
	  "  [--master BDADDR]   Pair USB devices with this adapter\n"
	  "  [--timestamp]       Print timestamp\n"
	  "  [--force-ds3]       Force detection of DS3\n"
	  "  [--verbose]         Print debugging information\n"
	  "  [--repeat-dump N]   Try to write all reports (dangerous)\n"
	  "  [--poll-report N]   Poll (ReportType<<8 | ReportID)\n"
	  "  [--nostdin]         Do not read from standard input\n");
  exit(1);
}

int main(int argc, char *argv[]) {
  struct motion_dev *devs = NULL;
  ds3 = malloc(sizeof(struct ds3_st));
  int next_devindex = 0;

  int csk = l2cap_listen(BDADDR_ANY, L2CAP_PSM_HIDP_CTRL);
  int isk = l2cap_listen(BDADDR_ANY, L2CAP_PSM_HIDP_INTR);
  if ( csk>=0 && isk>=0 )
    fprintf(stderr, "Waiting for Bluetooth connections.\n");
  else
  {
    fprintf(stderr, "Unable to listen on HID PSMs."
	    " Exiting\n");
    return -1;
  }
  for ( int i=1; i<argc; ++i )
    if ( ! strcmp(argv[i], "--master") && i+1<argc )
      usb_force_master = argv[++i];
    else if ( ! strcmp(argv[i], "--ir") )            enable_ir = 1;
    else if ( ! strcmp(argv[i], "--wmp") )           enable_wmp = 1;
    else if ( ! strcmp(argv[i], "--rgb") && i+1<argc &&
	      sscanf(argv[i+1],"%d,%d,%d",
		     &def_rgb[0],&def_rgb[1],&def_rgb[2])==3 ) ++i;
    else if ( ! strcmp(argv[i], "--timestamp") )     timestamp = 1;
    else if ( ! strcmp(argv[i], "--force-ds3") )     force_ds3 = 1;
    else if ( ! strcmp(argv[i], "--text") )          output_text = 1;
    else if ( ! strcmp(argv[i], "--verbose") )       verbose = 1;
    else if ( ! strcmp(argv[i], "--poll-report") && i+1<argc )
      poll_report = strtol(argv[++i], NULL, 0);
    else if ( ! strcmp(argv[i], "--nostdin") ) nostdin = 1;
    else {
      bdaddr_t ba;
      if ( mystr2ba(argv[i], &ba) ) usage();
    }

  usb_scan();

  output_text = 1;

  while ( 1 ) {
    fd_set fds; FD_ZERO(&fds);
    if ( ! nostdin ) FD_SET(0, &fds);
    int fdmax = 0;
    if ( csk >= 0 ) FD_SET(csk, &fds);
    if ( isk >= 0 ) FD_SET(isk, &fds);
    if ( csk > fdmax ) fdmax = csk;
    if ( isk > fdmax ) fdmax = isk;
    for ( struct motion_dev *dev=devs; dev; dev=dev->next ) {
      FD_SET(dev->csk, &fds); if ( dev->csk > fdmax ) fdmax = dev->csk;
      FD_SET(dev->isk, &fds); if ( dev->isk > fdmax ) fdmax = dev->isk;
    }
    if ( select(fdmax+1,&fds,NULL,NULL,NULL) < 0 ) fatal("select");
    struct timeval tv;
    gettimeofday(&tv, NULL);
    // Incoming connection ?
    if ( csk>=0 && FD_ISSET(csk,&fds) ) {
      struct motion_dev *dev = accept_device(csk, isk);
      dev->index = next_devindex++;
      dev->next = devs;
      devs = dev;
      setup_device(dev);
    }
    // Incoming input report ?
    for ( struct motion_dev *dev=devs; dev; dev=dev->next )
      if ( FD_ISSET(dev->isk, &fds) ) {
	unsigned char report[256];
	int nr = recv(dev->isk, report, sizeof(report), 0);
	if ( nr <= 0 ) {
	  fprintf(stderr, "%d disconnected\n", dev->index);
	  close(dev->csk); close(dev->isk);
	  struct motion_dev **pdev;
	  for ( pdev=&devs; *pdev!=dev; pdev=&(*pdev)->next ) ;
	  *pdev = dev->next;
	  free(dev);
	} else {
	  if ( verbose ) dump("RECV", report, nr);
	  if ( report[0] == 0xa1 ) {
	      parse_report(dev, report+1, nr-1);
	      fflush(stdout);
	  }
	}
      }

    // User command on stdin ?
    if ( FD_ISSET(0, &fds) ) {
      char line[256];
      if ( ! fgets(line, sizeof(line), stdin) ) return 0;
    }
  }
  free(ds3);
  return 0;
}
