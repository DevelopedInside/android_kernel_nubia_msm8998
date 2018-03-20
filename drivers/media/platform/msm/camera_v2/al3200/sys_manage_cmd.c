/*
 * File:  sys_managec_md.c
 * Description: System manage command
 *
 * (C)Copyright altek Corporation 2013
 *
 *  2013/10/14; Aaron Chuang; Initial version
 */
/* Standard C*/

/* Global Header Files*/
/*#include <osshell.h>*/

#include "include/isp_camera_cmd.h"
/* ISP Ctrl IF slave*/
#include "include/ispctrl_if_master.h"
/* ISP Ctrl IF slave error*/
#include "include/error/ispctrl_if_master_err.h"

/* Local Header Files*/
#include "include/ispctrl_if_master_local.h"



/******Include File******/



/******Private Constant Definition******/


#define MINI_ISP_LOG_TAG "[sys_manage_cmd]"


/******Private Type Declaration******/



/******Private Function Prototype******/

/******Private Global Variable******/



/******Public Global Variable******/

/******Public Function******/


/**
 *\brief Change mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_change_mode(struct misp_data *devdata,
					u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Parameter size*/
	u32 para_size = 1;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
					param, para_size);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_change_mode_end;

	if (ISPCTRL_POWERDOWN_MODE == param[0]) {
		/*wait for the interrupt*/
		err = mini_isp_wait_for_event(MINI_ISP_RCV_PWRDWN);
		if (err) {
			misp_err("%s - irq error: err : %d", __func__, err);
			goto mast_sys_manage_cmd_change_mode_end;
		}

	}

mast_sys_manage_cmd_change_mode_end:

	return err;


}

/**
 *\brief Get status of Mode Change
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_status_of_mode_change(
						struct misp_data *devdata,
							u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Parameter size*/
	u32 para_size = 1;



	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, param, 0);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_get_status_of_mode_change_end;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata, param,
						para_size, true);
mast_sys_manage_cmd_get_status_of_mode_change_end:

	return err;


}

/**
 *\brief Get Status of Last Executed Command
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_status_of_last_exec_command(
			struct misp_data *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Parameter size*/
	u32 para_size = 6;



	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, param, 0);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_get_status_of_last_exec_command_end;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata, param,
						para_size, true);
mast_sys_manage_cmd_get_status_of_last_exec_command_end:

	return err;


}

/**
 *\brief Get Error Code Command
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_error_code_command(struct misp_data *devdata,
							u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Parameter size*/
	u32 para_size = (sizeof(errcode))*10;



	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, param, 0);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_get_error_code_command_end;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata, param,
							para_size, false);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_get_error_code_command_end;

	misp_err("%s last error code %x %x %x %x", __func__, *(param),
 					*(param+1), *(param+2), *(param+3));
mast_sys_manage_cmd_get_error_code_command_end:

	return err;
}

/**
 *\brief Set ISP register
 *\param devdata [In], misp_data
 *\param opCode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_set_isp_register(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Parameter size*/
	u32 para_size = 8;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
						param, para_size);
	return err;
}

/**
 *\brief Get ISP register
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_isp_register(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Parameter size*/
	u32 para_size = 8;
	/* Response size*/
	u32 *reg_count = (u32 *)&param[4];



	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
						param, para_size);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_get_isp_register_end;

	/* Update response size*/
	para_size = 8 + *reg_count*4;

	/* Get data from slave*/
	err = ispctrl_if_mast_recv_isp_register_response_from_slave(
								devdata,
								param,
								&para_size,
								*reg_count);
mast_sys_manage_cmd_get_isp_register_end:

	return err;
}

/**
 *\brief Set common log level
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_set_comomn_log_level(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size = 4;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
					param, para_size);
	return err;
}

/**
 *\brief Get chip test report
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_chip_test_report(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size = 0;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
						param, para_size);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_get_chip_test_report_end;

	/* Update response size*/
	para_size = ReportRegCount;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata, param,
						para_size, true);
mast_sys_manage_cmd_get_chip_test_report_end:

	return err;
}

/**
 *\brief Get IRQ status
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_irq_status(struct misp_data *devdata,
						u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Parameter size*/
	u32 para_size = 2;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, param, 0);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_get_irq_status_end;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata, param,
						para_size, true);
mast_sys_manage_cmd_get_irq_status_end:

	return err;
}

/**
 *\brief Get Polling Command Status
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_polling_command_status(
						struct misp_data *devdata,
							u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	/* Parameter size*/
	u32 para_size = 1;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, param, 0);
	if (ERR_SUCCESS != err)
		goto mast_sys_manage_cmd_get_polling_command_status_end;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata, param,
							para_size, true);
mast_sys_manage_cmd_get_polling_command_status_end:

	return err;
}


/******End Of File******/
