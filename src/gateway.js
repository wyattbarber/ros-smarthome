const ros = require('rosnodejs');
const std_msgs = require('std_msgs').msg;
const smarthome_msgs = require('smarthome').msg;

const https = require('https');
const express = require('express');
const app = express();

const { exec } = require('child_process');

const nh = ros.initNode('smarthome_gateway');

// Global variables
var stored_ip;
const ip_store_url = '';
const cloud_secret = '209j1ncncsc8w0010nijsb0q';

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
            if (body.secret != cloud_secret) {
                ros.log.error('Incorrect secret from cloud.');
                return;
            }
            if (body.address != ip) {
                ros.log.error('Cloud function returned wrong IP.');
                return;
            }
            stored_ip = ip;
            ros.log.info('Updated connection to cloud functions.');
            return;
        })
    }
}





// Main program flow
if (require.main === module) {
    check_ip();
}