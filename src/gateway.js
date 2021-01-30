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
const ngrok = require('ngrok');
const https = require('https');
const express = require('express');
const app = express();
// Command line interface
const { exec } = require('child_process');

// Global data
const ngrokOpt = {
    proto: 'http',
    addr: 5050,
    onStatusChange: update_url(status)
};

// Called on ngrok status change. Restarts ngrok if tunnel has expires, gives url to cloud functions
async function update_url(status) {
    // Restart ngrok
    public_url = await ngrok.connect(ngrokOpt);

    // Store url in cloud functions
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
    update_url('closed');
}