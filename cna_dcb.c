#include <linux/dcbnl.h>
#include "cna_dcb.h"

#define BIT_DCB_MODE	0x01
#define BIT_PFC		0x02
#define BIT_PG_RX	0x04
#define BIT_PG_TX	0x08
#define BIT_APP_UPCHG	0x10
#define BIT_RESETLINK	0x40
#define BIT_LINKSPEED	0x80

/* Responses for the DCB_C_SET_ALL command */
#define DCB_HW_CHG_RST	0  /* DCB configuration changed with reset */
#define DCB_NO_HW_CHG	1  /* DCB configuration did not change */
#define DCB_HW_CHG	2  /* DCB configuration changed, no reset */

struct ixgbe_dcb_config temp_dcb_cfg;
struct ixgbe_dcb_config dcb_cfg;
u8 dcb_set_bitmap;
u8 fcoe_up;
u8 fcoe_up_set;

int ixgbe_copy_dcb_cfg(int tc_max)
{
	struct ixgbe_dcb_config *scfg = &temp_dcb_cfg;
	struct ixgbe_dcb_config *dcfg = &dcb_cfg;
	struct ixgbe_dcb_tc_config *src = NULL;
	struct ixgbe_dcb_tc_config *dst = NULL;
	int i, j;
	int tx = 0;
	int rx = 1;
	int changes = 0;

	if (fcoe_up_set != fcoe_up)
		changes |= BIT_APP_UPCHG;

	for (i = DCB_PG_ATTR_TC_0; i < tc_max + DCB_PG_ATTR_TC_0; i++) {
		src = &scfg->tc_config[i - DCB_PG_ATTR_TC_0];
		dst = &dcfg->tc_config[i - DCB_PG_ATTR_TC_0];

		if (dst->path[tx].tsa != src->path[tx].tsa) {
			dst->path[tx].tsa = src->path[tx].tsa;
			changes |= BIT_PG_TX;
		}

		if (dst->path[tx].bwg_id != src->path[tx].bwg_id) {
			dst->path[tx].bwg_id = src->path[tx].bwg_id;
			changes |= BIT_PG_TX;
		}

		if (dst->path[tx].bwg_percent != src->path[tx].bwg_percent) {
			dst->path[tx].bwg_percent = src->path[tx].bwg_percent;
			changes |= BIT_PG_TX;
		}

		if (dst->path[tx].up_to_tc_bitmap !=
		    src->path[tx].up_to_tc_bitmap) {
			dst->path[tx].up_to_tc_bitmap =
				src->path[tx].up_to_tc_bitmap;
			changes |= (BIT_PG_TX | BIT_PFC | BIT_APP_UPCHG);
		}

		if (dst->path[rx].tsa != src->path[rx].tsa) {
			dst->path[rx].tsa = src->path[rx].tsa;
			changes |= BIT_PG_RX;
		}

		if (dst->path[rx].bwg_id != src->path[rx].bwg_id) {
			dst->path[rx].bwg_id = src->path[rx].bwg_id;
			changes |= BIT_PG_RX;
		}

		if (dst->path[rx].bwg_percent != src->path[rx].bwg_percent) {
			dst->path[rx].bwg_percent = src->path[rx].bwg_percent;
			changes |= BIT_PG_RX;
		}

		if (dst->path[rx].up_to_tc_bitmap !=
		    src->path[rx].up_to_tc_bitmap) {
			dst->path[rx].up_to_tc_bitmap =
				src->path[rx].up_to_tc_bitmap;
			changes |= (BIT_PG_RX | BIT_PFC | BIT_APP_UPCHG);
		}
	}

	for (i = DCB_PG_ATTR_BW_ID_0; i < DCB_PG_ATTR_BW_ID_MAX; i++) {
		j = i - DCB_PG_ATTR_BW_ID_0;

		if (dcfg->bw_percentage[tx][j] != scfg->bw_percentage[tx][j]) {
			dcfg->bw_percentage[tx][j] = scfg->bw_percentage[tx][j];
			changes |= BIT_PG_TX;
		}
		if (dcfg->bw_percentage[rx][j] != scfg->bw_percentage[rx][j]) {
			dcfg->bw_percentage[rx][j] = scfg->bw_percentage[rx][j];
			changes |= BIT_PG_RX;
		}
	}

	for (i = DCB_PFC_UP_ATTR_0; i < DCB_PFC_UP_ATTR_MAX; i++) {
		j = i - DCB_PFC_UP_ATTR_0;
		if (dcfg->tc_config[j].pfc != scfg->tc_config[j].pfc) {
			dcfg->tc_config[j].pfc = scfg->tc_config[j].pfc;
			changes |= BIT_PFC;
		}
	}

	if (dcfg->pfc_mode_enable != scfg->pfc_mode_enable) {
		dcfg->pfc_mode_enable = scfg->pfc_mode_enable;
		changes |= BIT_PFC;
	}

	return changes;
}

s32 ixgbe_dcb_calculate_tc_credits_cee(struct ixgbe_dcb_config *dcb_config, u8 direction)
{
	struct ixgbe_dcb_tc_path *p;
	u32 min_multiplier	= 0;
	u16 min_percent		= 100;
	s32 ret_val =		0;

	u16 link_percentage	= 0;
	u8  bw_percent		= 0;
	u8  i;

	if (dcb_config == NULL) {
		ret_val = -1;
		goto out;
	}

	/* Find smallest link percentage */
	for (i = 0; i < 8; i++) {
		p = &dcb_config->tc_config[i].path[direction];
		bw_percent = dcb_config->bw_percentage[direction][p->bwg_id];
		link_percentage = p->bwg_percent;

		link_percentage = (link_percentage * bw_percent) / 100;

		if (link_percentage && link_percentage < min_percent)
			min_percent = link_percentage;
	}

	/* Find out the link percentage for each TC first */
	for (i = 0; i < 8; i++) {
		p = &dcb_config->tc_config[i].path[direction];
		bw_percent = dcb_config->bw_percentage[direction][p->bwg_id];

		link_percentage = p->bwg_percent;
		/* Must be careful of integer division for very small nums */
		link_percentage = (link_percentage * bw_percent) / 100;
		if (p->bwg_percent > 0 && link_percentage == 0)
			link_percentage = 1;

		/* Save link_percentage for reference */
		p->link_percent = (u8)link_percentage;
	}

out:
	return ret_val;
}

void ixgbe_dcb_unpack_bwgid_cee(struct ixgbe_dcb_config *cfg, int direction,
			    u8 *bwgid)
{
	struct ixgbe_dcb_tc_config *tc_config = &cfg->tc_config[0];
	int tc;

	for (tc = 0; tc < 8; tc++)
		bwgid[tc] = tc_config[tc].path[direction].bwg_id;
}

void ixgbe_dcb_unpack_tsa_cee(struct ixgbe_dcb_config *cfg, int direction,
			   u8 *tsa)
{
	struct ixgbe_dcb_tc_config *tc_config = &cfg->tc_config[0];
	int tc;

	for (tc = 0; tc < 8; tc++)
		tsa[tc] = tc_config[tc].path[direction].tsa;
}

void ixgbe_dcb_unpack_map_cee(struct ixgbe_dcb_config *cfg, int direction,
			      u8 *map)
{
	u8 up;

	for (up = 0; up < 8; up++)
		map[up] = ixgbe_dcb_get_tc_from_up(cfg, direction, up);
}

void ixgbe_dcb_unpack_pfc_cee(struct ixgbe_dcb_config *cfg, u8 *map, u8 *pfc_up)
{
	struct ixgbe_dcb_tc_config *tc_config = &cfg->tc_config[0];
	int up;

	/*
	 * If the TC for this user priority has PFC enabled then set the
	 * matching bit in 'pfc_up' to reflect that PFC is enabled.
	 */
	for (*pfc_up = 0, up = 0; up < 8; up++) {
		if (tc_config[map[up]].pfc != ixgbe_dcb_pfc_disabled)
			*pfc_up |= 1 << up;
	}
}

s32 ixgbe_dcb_hw_config_cee(struct ixgbe_dcb_config *dcb_config)
{
	s32 ret = 0x7FFFFFFF;
	u8 pfc_en;
	u8 tsa[8];
	u8 bwgid[8];
	u8 map[8] = { 0 };
	u16 max[8];

	ixgbe_dcb_unpack_bwgid_cee(dcb_config, 0, bwgid);
	ixgbe_dcb_unpack_tsa_cee(dcb_config, 0, tsa);
	ixgbe_dcb_unpack_map_cee(dcb_config, 0, map);

	return ret;
}


static u8 ixgbe_dcbnl_get_state(struct net_device *netdev)
{
	return 1;
}

static u8 ixgbe_dcbnl_set_state(struct net_device *netdev, u8 state){
	netdev_set_num_tc(netdev, 8);
	return 0;
}

static void ixgbe_dcbnl_get_perm_hw_addr(struct net_device *netdev,
					 u8 *perm_addr)
{
	int i;

	memset(perm_addr, 0xff, MAX_ADDR_LEN);
	for (i = 0; i < netdev->addr_len; i++)
		perm_addr[i] = netdev->dev_addr[i];
}

static void ixgbe_dcbnl_set_pg_tc_cfg_tx(struct net_device *netdev, int tc,
					 u8 prio, u8 bwg_id, u8 bw_pct,
					 u8 up_map)
{
	if (prio != DCB_ATTR_VALUE_UNDEFINED)
		temp_dcb_cfg.tc_config[tc].path[0].tsa = prio;
	if (bwg_id != DCB_ATTR_VALUE_UNDEFINED)
		temp_dcb_cfg.tc_config[tc].path[0].bwg_id = bwg_id;
	if (bw_pct != DCB_ATTR_VALUE_UNDEFINED)
		temp_dcb_cfg.tc_config[tc].path[0].bwg_percent =
			bw_pct;
	if (up_map != DCB_ATTR_VALUE_UNDEFINED)
		temp_dcb_cfg.tc_config[tc].path[0].up_to_tc_bitmap =
			up_map;
}

static void ixgbe_dcbnl_set_pg_bwg_cfg_tx(struct net_device *netdev, int bwg_id,
					  u8 bw_pct)
{
	temp_dcb_cfg.bw_percentage[0][bwg_id] = bw_pct;
}

static void ixgbe_dcbnl_set_pg_tc_cfg_rx(struct net_device *netdev, int tc,
					 u8 prio, u8 bwg_id, u8 bw_pct,
					 u8 up_map)
{
	if (prio != DCB_ATTR_VALUE_UNDEFINED)
		temp_dcb_cfg.tc_config[tc].path[1].tsa = prio;
	if (bwg_id != DCB_ATTR_VALUE_UNDEFINED)
		temp_dcb_cfg.tc_config[tc].path[1].bwg_id = bwg_id;
	if (bw_pct != DCB_ATTR_VALUE_UNDEFINED)
		temp_dcb_cfg.tc_config[tc].path[1].bwg_percent =
			bw_pct;
	if (up_map != DCB_ATTR_VALUE_UNDEFINED)
		temp_dcb_cfg.tc_config[tc].path[1].up_to_tc_bitmap =
			up_map;
}

static void ixgbe_dcbnl_set_pg_bwg_cfg_rx(struct net_device *netdev, int bwg_id,
					  u8 bw_pct)
{
	temp_dcb_cfg.bw_percentage[1][bwg_id] = bw_pct;
}

static void ixgbe_dcbnl_get_pg_tc_cfg_tx(struct net_device *netdev, int tc,
					 u8 *prio, u8 *bwg_id, u8 *bw_pct,
					 u8 *up_map)
{
	*prio = dcb_cfg.tc_config[tc].path[0].tsa;
	*bwg_id = dcb_cfg.tc_config[tc].path[0].bwg_id;
	*bw_pct = dcb_cfg.tc_config[tc].path[0].bwg_percent;
	*up_map = dcb_cfg.tc_config[tc].path[0].up_to_tc_bitmap;
}

static void ixgbe_dcbnl_get_pg_bwg_cfg_tx(struct net_device *netdev, int bwg_id,
					  u8 *bw_pct)
{
	*bw_pct = dcb_cfg.bw_percentage[0][bwg_id];
}

static void ixgbe_dcbnl_get_pg_tc_cfg_rx(struct net_device *netdev, int tc,
					 u8 *prio, u8 *bwg_id, u8 *bw_pct,
					 u8 *up_map)
{
	*prio = dcb_cfg.tc_config[tc].path[1].tsa;
	*bwg_id = dcb_cfg.tc_config[tc].path[1].bwg_id;
	*bw_pct = dcb_cfg.tc_config[tc].path[1].bwg_percent;
	*up_map = dcb_cfg.tc_config[tc].path[1].up_to_tc_bitmap;
}

static void ixgbe_dcbnl_get_pg_bwg_cfg_rx(struct net_device *netdev, int bwg_id,
					  u8 *bw_pct)
{

	*bw_pct = dcb_cfg.bw_percentage[1][bwg_id];
}

u8 ixgbe_dcb_get_tc_from_up(struct ixgbe_dcb_config *cfg, int direction, u8 up)
{
	struct ixgbe_dcb_tc_config *tc_config = &cfg->tc_config[0];
	u8 prio_mask = 1 << up;
	u8 tc = cfg->num_tcs.pg_tcs;

	/* If tc is 0 then DCB is likely not enabled or supported */
	if (!tc)
		goto out;

	for (tc--; tc; tc--) {
		if (prio_mask & tc_config[tc].path[direction].up_to_tc_bitmap)
			break;
	}
out:
	return tc;
}

static void ixgbe_dcbnl_set_pfc_cfg(struct net_device *netdev, int up, u8 pfc)
{
	u8 tc = ixgbe_dcb_get_tc_from_up(&temp_dcb_cfg, 0, up);

	temp_dcb_cfg.tc_config[tc].pfc = pfc;
	if (temp_dcb_cfg.tc_config[tc].pfc !=
	    dcb_cfg.tc_config[tc].pfc)
		temp_dcb_cfg.pfc_mode_enable = true;
}

static void ixgbe_dcbnl_get_pfc_cfg(struct net_device *netdev, int up, u8 *pfc)
{
	u8 tc = ixgbe_dcb_get_tc_from_up(&dcb_cfg, 0, up);
	*pfc = dcb_cfg.tc_config[tc].pfc;
}

static u8 ixgbe_dcbnl_set_all(struct net_device *netdev)
{
	int ret = DCB_NO_HW_CHG;
	u8 prio_tc[8] = { 0 };
	u8 i;

	dcb_set_bitmap |= ixgbe_copy_dcb_cfg(8);
	if (dcb_set_bitmap)
		return ret;

	ixgbe_dcb_unpack_map_cee(&dcb_cfg, 0, prio_tc);
	
	ixgbe_dcb_calculate_tc_credits_cee(&dcb_cfg,0);
	ixgbe_dcb_calculate_tc_credits_cee(&dcb_cfg,1);
	ixgbe_dcb_hw_config_cee(&dcb_cfg);
	
	if (dcb_set_bitmap & (BIT_PG_TX | BIT_PG_RX)) {

		for (i = 0; i < IEEE_8021QAZ_MAX_TCS; i++)
			netdev_set_prio_tc_map(netdev, i, prio_tc[i]);
		ret = DCB_HW_CHG_RST;
	}

	if (dcb_set_bitmap & BIT_PFC) {
		if (dcb_cfg.pfc_mode_enable) {
			u8 pfc_en;
			ixgbe_dcb_unpack_pfc_cee(&dcb_cfg, prio_tc, &pfc_en);
		} 
		if (ret != DCB_HW_CHG_RST)
			ret = DCB_HW_CHG;
	}

	if (dcb_set_bitmap & BIT_APP_UPCHG) {
		fcoe_up_set = fcoe_up;
		ret = DCB_HW_CHG_RST;
	}

	dcb_set_bitmap = 0x00;
	return ret;
}

static u8 ixgbe_dcbnl_getcap(struct net_device *netdev, int capid, u8 *cap)
{
	switch (capid) {
	case DCB_CAP_ATTR_PG:
		*cap = true;
		break;
	case DCB_CAP_ATTR_PFC:
		*cap = true;
		break;
	case DCB_CAP_ATTR_UP2TC:
		*cap = false;
		break;
	case DCB_CAP_ATTR_PG_TCS:
		*cap = 0x80;
		break;
	case DCB_CAP_ATTR_PFC_TCS:
		*cap = 0x80;
		break;
	case DCB_CAP_ATTR_GSP:
		*cap = true;
		break;
	case DCB_CAP_ATTR_BCN:
		*cap = false;
		break;
	default:
		*cap = false;
		break;
	}

	return 0;
}

static int ixgbe_dcbnl_getnumtcs(struct net_device *netdev, int tcid, u8 *num)
{
	u8 rval = 0;
	
	switch (tcid) {
		case DCB_NUMTCS_ATTR_PG:
			*num = dcb_cfg.num_tcs.pg_tcs;
			break;
		case DCB_NUMTCS_ATTR_PFC:
			*num = dcb_cfg.num_tcs.pfc_tcs;
			break;
		default:
			rval = -5;
			break;
	}
	
	return rval;
}

static int ixgbe_dcbnl_setnumtcs(struct net_device *netdev, int tcid, u8 num)
{
	u8 rval = 0;

	switch (tcid) {
		case DCB_NUMTCS_ATTR_PG:
			dcb_cfg.num_tcs.pg_tcs = num;
			break;
		case DCB_NUMTCS_ATTR_PFC:
			dcb_cfg.num_tcs.pfc_tcs = num;
			break;
		default:
			rval = -EINVAL;
			break;
		}

	return rval;
}

static u8 ixgbe_dcbnl_getpfcstate(struct net_device *netdev)
{
	return dcb_cfg.pfc_mode_enable;
}

static void ixgbe_dcbnl_setpfcstate(struct net_device *netdev, u8 state)
{
	temp_dcb_cfg.pfc_mode_enable = state;
	return;
}

static u8 ixgbe_dcbnl_getapp(struct net_device *netdev, u8 idtype, u16 id)
{
	u8 rval = 0;

	switch (idtype) {
	case DCB_APP_IDTYPE_ETHTYPE:
		if (id == ETH_P_FCOE)
			rval = 1<<fcoe_up;
		break;
	case DCB_APP_IDTYPE_PORTNUM:
		break;
	default:
		break;
	}

	return rval;
}

static u8 ixgbe_dcbnl_setapp(struct net_device *netdev,
			     u8 idtype, u16 id, u8 up)
{
	int err = 0;

	switch (idtype) {
	case DCB_APP_IDTYPE_ETHTYPE:
		if (id == ETH_P_FCOE) {
			fcoe_up = ffs(up) - 1;
		}
		break;
	case DCB_APP_IDTYPE_PORTNUM:
		break;
	default:
		break;
	}

	return err;
}

static u8 nf10_dcbnl_getdcbx(struct net_device *dev)
{
//to add somethig later
	return 1;

}

struct dcbnl_rtnl_ops dcbnl_ops = {
	.getstate	= ixgbe_dcbnl_get_state,
	.setstate	= ixgbe_dcbnl_set_state,
	.getpermhwaddr	= ixgbe_dcbnl_get_perm_hw_addr,
	.setpgtccfgtx	= ixgbe_dcbnl_set_pg_tc_cfg_tx,
	.setpgbwgcfgtx	= ixgbe_dcbnl_set_pg_bwg_cfg_tx,
	.setpgtccfgrx	= ixgbe_dcbnl_set_pg_tc_cfg_rx,
	.setpgbwgcfgrx	= ixgbe_dcbnl_set_pg_bwg_cfg_rx,
	.getpgtccfgtx	= ixgbe_dcbnl_get_pg_tc_cfg_tx,
	.getpgbwgcfgtx	= ixgbe_dcbnl_get_pg_bwg_cfg_tx,
	.getpgtccfgrx	= ixgbe_dcbnl_get_pg_tc_cfg_rx,
	.getpgbwgcfgrx	= ixgbe_dcbnl_get_pg_bwg_cfg_rx,
	.setpfccfg	= ixgbe_dcbnl_set_pfc_cfg,
	.getpfccfg	= ixgbe_dcbnl_get_pfc_cfg,
	.setall		= ixgbe_dcbnl_set_all,
	.getcap		= ixgbe_dcbnl_getcap,
	.getnumtcs	= ixgbe_dcbnl_getnumtcs,
	.setnumtcs	= ixgbe_dcbnl_setnumtcs,
	.getpfcstate	= ixgbe_dcbnl_getpfcstate,
	.setpfcstate	= ixgbe_dcbnl_setpfcstate,
	.getapp		= ixgbe_dcbnl_getapp,
	.setapp		= ixgbe_dcbnl_setapp,
	.getdcbx	= nf10_dcbnl_getdcbx,
};



void nf10_dcb_init(struct net_device *netdev){
	struct ixgbe_dcb_tc_config *tc;
	int j, bwg_pct;
	
	netdev->dcbnl_ops = &dcbnl_ops;
	fcoe_up = 0x3;
	fcoe_up_set = 0x3;
	dcb_cfg.num_tcs.pg_tcs = 8;
	dcb_cfg.num_tcs.pfc_tcs = 8;

	bwg_pct = 100 / dcb_cfg.num_tcs.pg_tcs;
	printk("dcb_init\n");
	for (j = 0; j < dcb_cfg.num_tcs.pg_tcs; j++) {
		tc = &dcb_cfg.tc_config[j];
		tc->path[0].bwg_id = 0;
		tc->path[0].bwg_percent = bwg_pct;
		tc->path[1].bwg_id = 0;
		tc->path[1].bwg_percent = bwg_pct;
		tc->pfc = ixgbe_dcb_pfc_disabled;
	}

	/* reset back to TC 0 */
	tc = &dcb_cfg.tc_config[0];

	/* total of all TCs bandwidth needs to be 100 */
	bwg_pct += 100 % dcb_cfg.num_tcs.pg_tcs;
	tc->path[0].bwg_percent = bwg_pct;
	tc->path[1].bwg_percent = bwg_pct;

	/* Initialize default user to priority mapping, UPx->TC0 */
	tc->path[0].up_to_tc_bitmap = 0xFF;
	tc->path[1].up_to_tc_bitmap = 0xFF;

	dcb_cfg.bw_percentage[0][0] = 100;
	dcb_cfg.bw_percentage[1][0] = 100;
	dcb_cfg.rx_pba_cfg = ixgbe_dcb_pba_equal;
	dcb_cfg.pfc_mode_enable = false;
	dcb_cfg.round_robin_enable = false;
	dcb_set_bitmap = 0x00;
	memcpy(&temp_dcb_cfg, &dcb_cfg,
	       sizeof(temp_dcb_cfg));

	ixgbe_dcb_calculate_tc_credits_cee(&dcb_cfg,0);
	ixgbe_dcb_calculate_tc_credits_cee(&dcb_cfg,1);
	ixgbe_dcb_hw_config_cee(&dcb_cfg);

	
}
