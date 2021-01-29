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

const { exec } = require('child_process');



// Global variables
var stored_ip;
const secret = '209j1ncncsc8w0010nijsb0q';
var ip_store_url = 'https://www.google.com/url?q=https%3A%2F%2Fus-central1-decent-booster-285122.cloudfunctions.net%2Fconnection_refresh';
ip_store_url += '?key=' + secret;
ip_store_url += '%26client_id=' + '12955530';

function check_ip() {
    // Get public ip address
    let ip;
    exec("curl ifconfig.me", (error, stdout, stderr) => {
        if (error) {
            ros.log.error('Error getting public IP: ' + error.message);
            return;
        }
        if (stderr) {
            ros.log.error('Error getting public IP: ' + stderr);
            return;
        }
        ip = stdout;
    });
    // Compare to ip last sent to google cloud
    if (ip != stored_ip) {
        ros.log.info('IP has changed to '+ip);
        // Send request to cloud function
        https.get(ip_store_url, (res) => {
            // Assemble response
            var body = '';
            let response;
            res.on('data', (chunk) => {
                body += chunk;
            });
            res.on('end', () => {
                response = JSON.parse(body);
            });
            res.on('error', (e) => {
                ros.log.error('Error parsing cloud function response: ' + e);
            });
            // Verify response data
            if (body.ip != ip) {
                ros.log.error('Cloud function returned wrong IP.');
                return;
            }
            stored_ip = ip;
            // TODO: Send key to cloud for sending smarthome intents
            ros.log.info('Updated connection to cloud functions.');
            return;
        })
    }
}





// Main program flow
if (require.main === module) {
    check_ip();
}