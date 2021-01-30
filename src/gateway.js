#!/usr/bin/env node

// ROS node to link to Google Cloud Function

// ROS setup
const ros = require('rosnodejs');
ros.initNode('smarthome_gateway')
    .then(nh => {
        const std_msgs = ros.require('std_msgs');
        const smarthome_msgs = ros.require('smarthome');
    });
// Web interfaces
const https = require('https');
const express = require('express');
const app = express();
// Command line interface
const { exec } = require('child_process');

// Global data
var stored_url = "";

// Function to get ngrok url and link with cloud if it has changed
function update_url() {
    // Check randomly generated url
    let public_url;
    exec("curl http://localhost:4040/api/tunnels", (error, stdout, stderr) => {
        try {
            // Got ngrok data, find https url
            const tunnels = JSON.parse(stdout);
            for (let t of tunnels.tunnels) {
                if (t.proto == 'https') {
                    public_url = t.public_url;
                    return;
                }
            }
        }
        catch (e) {
            ros.log.error('Error parsing ngrok response: '+e);
        }
    });

    // Compare url to that stored in cloud, update cloud if neccessary
    if (public_url === stored_url) {
        return; // No action needed
    }
    ros.log.info('Public url has changed to: ' + public_url);
    const options = {
        hostname: 'https://us-central1-decent-booster-285122.cloudfunctions.net',
        path: '/connection_refresh?secret=209j1ncncsc8w0010nijsb0q&url='+public_url+'&client_id=12955530',
        method: 'POST'
    }
    const req = https.request(options, (res) => {
        let d, data;
        res.on('data', (chunk) => {d+=chunk;});
        res.on('end', () => {data = JSON.parse(d);});
        if (data.body.key != 'nqcpn-93gbwbwoc01') {
            ros.log.error('Could not verify response identity');
            return;
        }
        if (data.body.url != public_url) {
            ros.log.error('Could not verify url stored in cloud');
            return;
        }
        stored_url = public_url;
        ros.log.info('Updated connection to cloud functions');
    });
    req.on('error', e => {ros.log.error('Error in url update cloud function: '+e);});
}

// Main program flow
if (require.main === module) {
    update_url();
}