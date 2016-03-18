#include <linux/netdevice.h>

enum ixgbe_dcb_tsa {
	ixgbe_dcb_tsa_ets = 0,
	ixgbe_dcb_tsa_group_strict_cee,
	ixgbe_dcb_tsa_strict
};

struct ixgbe_dcb_tc_path {
	u8 bwg_id; /* Bandwidth Group (BWG) ID */
	u8 bwg_percent; /* % of BWG's bandwidth */
	u8 link_percent; /* % of link bandwidth */
	u8 up_to_tc_bitmap; /* User Priority to Traffic Class mapping */
	u16 data_credits_refill; /* Credit refill amount in 64B granularity */
	u16 data_credits_max; /* Max credits for a configured packet buffer
			       * in 64B granularity.*/
	enum ixgbe_dcb_tsa tsa; /* Link or Group Strict Priority */
};

enum ixgbe_dcb_pfc {
	ixgbe_dcb_pfc_disabled = 0,
	ixgbe_dcb_pfc_enabled,
	ixgbe_dcb_pfc_enabled_txonly,
	ixgbe_dcb_pfc_enabled_rxonly
};

struct ixgbe_dcb_tc_config {
	struct ixgbe_dcb_tc_path path[2]; /* One each for Tx/Rx */
	enum ixgbe_dcb_pfc pfc; /* Class based flow control setting */

	u16 desc_credits_max; /* For Tx Descriptor arbitration */
	u8 tc; /* Traffic class (TC) */
};

struct ixgbe_dcb_support {
	u32 capabilities; /* DCB capabilities */

	/* Each bit represents a number of TCs configurable in the hw.
	 * If 8 traffic classes can be configured, the value is 0x80. */
	u8 traffic_classes;
	u8 pfc_traffic_classes;
};

enum ixgbe_dcb_pba {
	/* PBA[0-7] each use 64KB FIFO */
	ixgbe_dcb_pba_equal = 0,
	/* PBA[0-3] each use 80KB, PBA[4-7] each use 48KB */
	ixgbe_dcb_pba_80_48 = 1
};

struct ixgbe_dcb_num_tcs {
	u8 pg_tcs;
	u8 pfc_tcs;
};

struct ixgbe_dcb_config {
	struct ixgbe_dcb_tc_config tc_config[8];
	struct ixgbe_dcb_support support;
	struct ixgbe_dcb_num_tcs num_tcs;
	u8 bw_percentage[2][8]; /* One each for Tx/Rx */
	bool pfc_mode_enable;
	bool round_robin_enable;

	enum ixgbe_dcb_pba rx_pba_cfg;

	u32 dcb_cfg_version; /* Not used...OS-specific? */
	u32 link_speed; /* For bandwidth allocation validation purpose */
	bool vt_mode;
};

void nf10_dcb_init(struct net_device *netdev);
u8 ixgbe_dcb_get_tc_from_up(struct ixgbe_dcb_config *cfg, int direction, u8 up);