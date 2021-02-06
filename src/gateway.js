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
const axios = require('axios');
const express = require('express');
const app = express();

// Command line interface
const { exec } = require('child_process');

app.post('/', (req, res) => {
  res.json({
    user: "1836.15267389",
    devices: [{
      id: "123",
      type: "action.devices.types.OUTLET",
      traits: [
        "action.devices.traits.OnOff"
      ],
      name: {
        defaultNames: ["My Outlet 1234"],
        name: "Night light",
        nicknames: ["wall plug"]
      },
      willReportState: false,
      roomHint: "kitchen",
      deviceInfo: {
        manufacturer: "lights-out-inc",
        model: "hs1234",
        hwVersion: "3.2",
        swVersion: "11.4"
      },
      otherDeviceIds: [{
        deviceId: "local-device-id"
      }],
      customData: {
        fooValue: 74,
        barValue: true,
        bazValue: "foo"
      }
    }]
  })
});


async function send_url() {
  let public_url;
  exec("curl http://localhost:4040/api/tunnels", (error, stdout, stderr) => {
    let data = JSON.parse(stdout);
    public_url = data.tunnels[0].public_url;
    console.log(public_url);

    axios({
      method: 'post',
      url: "https://us-central1-decent-booster-285122.cloudfunctions.net/connection_refresh",
      data: {
        url: public_url,
        client_id: '12955530',
        secret: "209j1ncncsc8w0010nijsb0q"
      }
    })
      .then((res) => {
        //console.log(res);
      })
      .catch((e) => {
        console.log(e);
      });
  });
};

send_url();
