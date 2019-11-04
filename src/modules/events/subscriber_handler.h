#pragma once

#include <chip.h>
#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vehicle_attitude.h>

namespace events
{

/**
 * @class SubscriberHandler
 * Contains a list of uORB subscriptions and maintains their update state.
 */
class SubscriberHandler
{
public:
	_EXT_ITCM void subscribe();
	_EXT_ITCM void unsubscribe();
	_EXT_ITCM void check_for_updates();

	_EXT_ITCM int get_battery_status_sub() const { return _battery_status_sub; }
	_EXT_ITCM int get_cpuload_sub() const { return _cpuload_sub; }
	_EXT_ITCM int get_vehicle_command_sub() const { return _vehicle_command_sub; }
	_EXT_ITCM int get_vehicle_status_sub() const { return _vehicle_status_sub; }
	_EXT_ITCM int get_vehicle_status_flags_sub() const { return _vehicle_status_flags_sub; }

	/* update checking methods */
	_EXT_ITCM bool battery_status_updated() const { return _update_bitfield & (uint32_t)StatusMask::BatteryStatus; }
	_EXT_ITCM bool cpuload_updated() const { return _update_bitfield & (uint32_t)StatusMask::CpuLoad; }
	_EXT_ITCM bool vehicle_command_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleCommand; }
	_EXT_ITCM bool vehicle_status_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleStatus; }
	_EXT_ITCM bool vehicle_status_flags_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleStatusFlags; }


private:
	enum class StatusMask : uint32_t {
		VehicleCommand = (0x01 << 0),
		VehicleStatus = (0x01 << 1),
		VehicleStatusFlags = (0x01 << 2),
		BatteryStatus = (0x01 << 3),
		CpuLoad = (0x01 << 4),
	};

	int _battery_status_sub = -1;
	int _cpuload_sub = -1;
	int _vehicle_command_sub = -1;
	int _vehicle_status_sub = -1;
	int _vehicle_status_flags_sub = -1;

	uint32_t _update_bitfield = 0;
};

} /* events */
