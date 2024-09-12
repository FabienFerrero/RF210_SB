
// Initialisation des objets avec des valeurs par défaut
var defaultDevices = {
    "rf210-sb00": [],
    "rf210-sb01": [],
    "rf210-sb02": [],
    "rf210-sb03": [],
    "rf210-sb04": [],
    "rf210-sb05": [],
    "rf210-sb06": [],
    "rf210-sb07": [],
    "rf210-sb08": [],
    "rf210-sb09": [],
    "rf210-sb10": [],
    "rf210-sb11": [],
    "rf210-sb12": [],
    "rf210-sb13": [],
    "rf210-sb14": [],
};



var detected_devices = flow.get('detected_devices') || { ...defaultDevices };
var iterations_detected_devices = flow.get('iterations_detected_devices') || { ...defaultDevices };
var time_measurement = flow.get('time_measurement') || { ...defaultDevices };
var rssi = flow.get('rssi') || { ...defaultDevices };


var paths = [];

var msg1 = {};
var msg2 = {};
var msg3 = {};

const PORT_NUMBER = 4;
const BYTES_PER_MESSAGE = 14;
const NUM_MAX_ITERATIONS = 15;

var deviceID = msg.measurement;
var flag_new_address = 0;


if (msg.payload.uplink_message.f_port == PORT_NUMBER) { // if the device sends a list of new detected devices (port 4)

    node.warn(msg.measurement);
    // get the data and convert into hexadecimal
    var detected_addresses = Buffer.from(msg.payload.uplink_message.frm_payload, 'base64').toString('hex');
    var nbad = detected_addresses.length / BYTES_PER_MESSAGE;

    //var created_path = false;
    for (let step = 0; step < nbad; step++) { // for each new address detected

        var flag_new_address = 0;
        var single_address = detected_addresses.slice(step * BYTES_PER_MESSAGE, step * BYTES_PER_MESSAGE + 12); // get the data and store it in single_address
        var single_rssi_level = detected_addresses.slice(step * BYTES_PER_MESSAGE + 12, step * BYTES_PER_MESSAGE + 14); // get the data and store it in single_rssi_level

        /* Step 1 : add the address to the list of the concerned current_device if it does not already belong to the device*/        

        if (detected_devices[deviceID].indexOf(single_address) == -1) { // if the flag was not activated, add it to the list of the concerned device 
            detected_devices[deviceID].push(single_address); // add it to the list
            iterations_detected_devices[deviceID].push(NUM_MAX_ITERATIONS); // add it to the list
            time_measurement[deviceID].push(msg.payload.uplink_message.decoded_payload.timex);
            rssi[deviceID].push(single_rssi_level);
            flag_new_address=1;
        } else {
            iterations_detected_devices[deviceID][detected_devices[deviceID].indexOf(single_address)] = NUM_MAX_ITERATIONS;
            rssi[deviceID][detected_devices[deviceID].indexOf(single_address)]=single_rssi_level;
        }

        /* Step 2: check if it already belongs to an other list and if so, creates a vector describing the path */

        // If a path is detected, the address detected is removed from every list except the list of the actual device
        // This way, only one device will have the address in the next iteration
        if (flag_new_address==0){
            for (let current_device in detected_devices) { // for each list
                if (current_device != deviceID) { // excepting the list of the current_device sending the message and his cluster
                    if (detected_devices[current_device].indexOf(single_address) > -1 ) { // if the address belongs to the list
                        
                        var index = detected_devices[current_device].indexOf(single_address); 
                        var index_deviceID = detected_devices[deviceID].indexOf(single_address);
                        var time_spent = msg.payload.uplink_message.decoded_payload.timex - time_measurement[current_device][index]; // time between paths
                        //node.warn("Date.now() = " + msg.payload.uplink_message.decoded_payload.timex);
                        //node.warn("Time_spent = " + time_spent);
                        //node.warn("Previous date.now = " +time_measurement[current_device][index] );
                        // create a path between both devices for the concerned address
                        paths.push({    
                            departure: current_device, 
                            arrival: deviceID,
                            rssi_departure: rssi[current_device][index],
                            rssi_arrival: rssi[deviceID][index_deviceID],
                            duration: time_spent, // XXXXXX to adapt, we consider here the hour of recepetion of the messages
                            hour: msg.payload.uplink_message.decoded_payload.timex, // XXXXXXX to adapt, we consider here the hour of reception of the message
                            address: single_address // we transmit the address of the device to detect if there are multiple paths for a single device
                        })
                        // example : path("FD45GDETGD34") = [1,4] : someone went from device 1 to device 4
                        detected_devices[current_device].splice( index, 1 ); // remove the address from the list
                        iterations_detected_devices[current_device].splice( index, 1 ); // remove the iterations of the address from the list
                        time_measurement[current_device].splice(index,1); // remove the time measurement from the list
                        rssi[current_device].splice(index,1);
                        break;

                    }
                }

            }
        }




    }




    var length_lists = {};
    var l = detected_devices.length;
    for (var device in detected_devices) {
        length_lists[device] = detected_devices[device].length;
    }

    flow.set("detected_devices", detected_devices);
    flow.set("time_measurement",time_measurement);
    flow.set("iterations_detected_devices",iterations_detected_devices);

/*
object = {
    departure : rf210-sb01 // arrival : rf210-sb02 // duration : 
    departure : 
}


*/

    msg1.payload = detected_devices; // lists of detected devices
    
    if (paths.length > 0) {
        msg2.payload = paths; // paths detected sent to influxDB
    }
    msg2.measurement = msg.measurement;
    msg3.payload = length_lists; // list of length of lists
    //node.warn("Detected addresses: " + JSON.stringify(detected_devices)); // Debug message
    //node.warn("paths detected: " + JSON.stringify(paths)); // Debug message
    //node.warn("time_measurements: " + JSON.stringify(time_measurement));
    node.warn("number of iterations: " + JSON.stringify(iterations_detected_devices)); // Debug message
    node.warn("rssi: " + JSON.stringify(rssi));

    //node.warn("Length of devices : " + JSON.stringify(length_lists));
    return [msg1, msg2, msg3];

}










