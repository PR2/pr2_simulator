#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PKG = 'pr2_gazebo_plugins'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord
import sys
import optparse
import numpy
import scipy.stats
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class IBPS:
    def __init__(self, name, id):
        self.name = name
        self.id   = id
        
        self.batteries = {}

class Battery:
    def __init__(self, name, ibps, id):
        self.name = name
        self.ibps = ibps
        self.id   = id
       
        self.remaining_capacity = TimeSeries()
        self.charging           = TimeSeries()
        self.voltage            = TimeSeries()
        self.current            = TimeSeries()
        
    def is_valid(self):
        for series in [self.remaining_capacity, self.charging, self.voltage, self.current]:
            if series.is_empty():
                return False

        return True
        
class TimeSeries:
    def __init__(self):
        self.points = []
        
    def append_point(self, stamp, value): self.points.append((stamp, value))
    def is_empty(self):                   return len(self.points) == 0
    def latest_value(self):               return self.points[-1][1]   
    def get_values(self):                 return [value for (stamp, value) in self.points]   
    def to_array(self):                   return numpy.array(self.get_values())

class PowerSystem:
    def __init__(self):
        self.ibpses = {}
        
        self.battery_count_history      = TimeSeries()
        self.charging_history           = TimeSeries()
        self.power_history              = TimeSeries()
        self.remaining_capacity_history = TimeSeries()
        self.mean_voltage_history       = TimeSeries()

    ## Reads DiagnosticStatus messages from the given bag file for performance history on each of the Smart Batteries 
    def append_bag_history(self, bag_path, stride=1, offset=0):
        for i, (topic, raw_msg, t) in enumerate(rosrecord.logplayer(bag_path, raw=True)):
            if rospy.is_shutdown():
                break
            
            if stride > 1 and (i - offset) % stride != 0:
                continue
            
            (datatype, message_data, md5, bag_pos, pytype) = raw_msg       
            if datatype != 'diagnostic_msgs/DiagnosticArray':
                continue
    
            msg = pytype()
            msg.deserialize(message_data)
            
            stamp = msg.header.stamp           

            battery_updated = False
            for status in msg.status:
                if status.name.startswith('Smart Battery'):
                    battery_name = status.name
    
                    # Get the ID of the IBPS and the battery
                    ids = battery_name.split(' ')[-1].split('.')
                    ibps_id, battery_id = int(ids[0]), int(ids[1]) 
                    
                    # Get the IBPS and battery objects (create if necessary)
                    ibps    = self.ibpses.setdefault(str(ibps_id), IBPS(str(ibps_id), ibps_id))
                    battery = ibps.batteries.setdefault(battery_name, Battery(battery_name, ibps, battery_id))
    
                    # Update the relevant timeseries
                    for kv in status.values:
                        if   kv.key == 'remainingCapacity (mAh)': series, value = battery.remaining_capacity, self.ocean_battery_reg_to_float(kv.value)
                        elif kv.key == 'charging':                series, value = battery.charging,           self.ocean_battery_reg_to_bool (kv.value)
                        elif kv.key == 'voltage (mV)':            series, value = battery.voltage,            self.ocean_battery_reg_to_float(kv.value)
                        elif kv.key == 'current (mA)':            series, value = battery.current,            self.ocean_battery_reg_to_float(kv.value)
                        else:                                     continue

                        series.append_point(stamp, value)
                        battery_updated = True

            if battery_updated:
                battery_count            = 0
                total_charging           = 0
                total_power              = 0.0
                total_remaining_capacity = 0.0
                total_voltage            = 0.0
                
                for ibps_id, ibps in self.ibpses.items():
                    for battery_id, battery in ibps.batteries.items():
                        if not battery.is_valid():
                            continue

                        battery_count += 1

                        if not battery.remaining_capacity.is_empty():
                            total_remaining_capacity += battery.remaining_capacity.latest_value()
                            
                        if not battery.charging.is_empty() and battery.charging.latest_value():
                            total_charging += 1
                            
                        if not battery.voltage.is_empty() and not battery.current.is_empty():
                            voltage = battery.voltage.latest_value()
                            current = battery.current.latest_value()
                            total_power += voltage * current
                            
                            total_voltage += voltage

                if battery_count > 0:
                    self.battery_count_history.append_point(stamp, battery_count)
                    self.charging_history.append_point(stamp, total_charging)
                    self.power_history.append_point(stamp, total_power)
                    self.remaining_capacity_history.append_point(stamp, total_remaining_capacity)
                    self.mean_voltage_history.append_point(stamp, total_voltage / battery_count)

    ## HACK: battery driver outputs diagnostics as raw values so we have to convert here (see wg-ros-pkg/#3052)
    @staticmethod
    def ocean_battery_reg_to_float(s):
        val = int(s)
        if val & 0x8000:
            val -= 65536
    
        return float(val) / 1000.0

    @staticmethod
    def ocean_battery_reg_to_bool(s):
        return s == 'True'

    ## Given histories of battery performance, estimate constant (dis)charge rates (in W) and voltages (in V) and a full charge capacity (in Ah)
    def estimate_characteristics(self, discharge_threshold, charge_threshold):
        if self.battery_count_history.is_empty():
            return None
        
        # Convert time series to numpy arrays
        battery_count_array      = self.battery_count_history.to_array()
        charging_array           = self.charging_history.to_array()
        power_array              = self.power_history.to_array()
        remaining_capacity_array = self.remaining_capacity_history.to_array()
        mean_voltage_array       = self.mean_voltage_history.to_array()

        # Ignore values if all batteries haven't reported yet 
        all_batteries_array      = (battery_count_array == numpy.max(battery_count_array))
        charging_array           = numpy.extract(all_batteries_array, charging_array)
        power_array              = numpy.extract(all_batteries_array, power_array)
        remaining_capacity_array = numpy.extract(all_batteries_array, remaining_capacity_array)
        mean_voltage_array       = numpy.extract(all_batteries_array, mean_voltage_array)

        # full charge capacity = max(remaining_capacity)
        if len(remaining_capacity_array) > 0:
            full_charge_capacity = numpy.max(remaining_capacity_array)
        else:
            full_charge_capacity = None

        # discharge rate = mode(power; charging == 0 and power < -discharge_threshold)
        discharge_power_array = numpy.extract(charging_array == 0, power_array)
        discharge_power_array = numpy.extract(discharge_power_array < discharge_threshold, discharge_power_array)
        if len(discharge_power_array) > 0:
            (bin_counts, bin_min, bin_width, outside_range) = scipy.stats.histogram(discharge_power_array, 20)
            discharge_power = bin_min + numpy.argmax(bin_counts) * bin_width
        else:
            discharge_power = None

        # charge rate = max(power; charging > 0 and power > charge_threshold)        
        charge_power_array = numpy.extract(charging_array > 0, power_array)
        charge_power_array = numpy.extract(charge_power_array > charge_threshold, charge_power_array)
        if len(charge_power_array) > 0:
            charge_power = numpy.max(charge_power_array)
        else:
            charge_power = None
            
        # discharge voltage = mean(voltage; charging == 0)
        discharge_mean_voltage_array = numpy.extract(charging_array == 0, mean_voltage_array)
        if len(discharge_mean_voltage_array) > 0:
            discharge_voltage = numpy.mean(discharge_mean_voltage_array)
        else:
            discharge_voltage = None
        
        # charge voltage = mean(voltage; charging > 0 and power > charge_threshold)
        charge_mean_voltage_array = numpy.extract(charging_array > 0, mean_voltage_array)
        if len(charge_mean_voltage_array) > 0:
            charge_voltage = numpy.mean(charge_mean_voltage_array)
        else:
            charge_voltage = None

        return (full_charge_capacity, discharge_power, charge_power, discharge_voltage, charge_voltage)

## Using diagnostic info from the given bag files, estimate battery performance characteristics
def measure_power(bag_paths, stride, discharge, charge):
    print '- parsing %d bag files' % len(bag_paths)
    print '- sampling every %d battery status message' % stride
    print '- discharge threshold of %.2f W' % discharge
    print '- charge threshold of %.2f W' % charge
    
    ps = PowerSystem()
    for i, bag_path in enumerate(bag_paths):
        print '[%d/%d] %s' % (i + 1, len(bag_paths), bag_path)

        # Read the bag
        ps.append_bag_history(bag_path, stride=options.stride)
        
        # Estimate the power characteristics from all bags so far
        characteristics = ps.estimate_characteristics(discharge, charge)
        if characteristics is None:
            print 'No data'
            continue
        
        # Print out the info
        (full_charge_capacity, discharge_power, charge_power, discharge_voltage, charge_voltage) = characteristics 
        if full_charge_capacity is not None:
            print '  full charge capacity: %.2f Ah' % full_charge_capacity
        if discharge_power is not None:
            print '  discharge power:      %d W' % discharge_power
        if charge_power is not None:
            print '  charge power:          %d W' % charge_power
        if discharge_voltage is not None:
            print '  discharge voltage:    %.2f V' % discharge_voltage
        if charge_voltage is not None:
            print '  charge voltage:       %.2f V' % charge_voltage

if __name__ == '__main__':
    # Parse command line
    parser = optparse.OptionParser()
    parser.add_option('-s', dest='stride',    default=  20, help='sample every Nth diagnostics message')
    parser.add_option('-d', dest='discharge', default=-200, help='level below which robot is considered discharging')
    parser.add_option('-c', dest='charge',    default= 200, help='level above which robot is considered charging')
    options, args = parser.parse_args(sys.argv[1:])
    bag_paths = args
    if len(bag_paths) == 0:
        parser.print_help()
    else:
        measure_power(bag_paths, options.stride, options.discharge, options.charge)
