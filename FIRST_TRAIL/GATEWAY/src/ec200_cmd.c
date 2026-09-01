/*
 * Downlink command receive for the EC200 - integration point.
 *
 * To make this real, add to ec200.c (which owns the modem UART + ec_rb):
 *
 *   In ec200_mqtt_up(), AFTER QMTCONN succeeds:
 *     1. Persistent session so the broker queues commands while we sleep:
 *          AT+QMTCFG="session",0,0        (0 = clean session disabled)
 *          (set BEFORE QMTOPEN/QMTCONN)
 *     2. Last-will (birth/LWT on the status topic):
 *          AT+QMTCFG="will",0,1,1,1,"smartwagon/v1/<wgn>/<gw>/status",
 *                    "{\"st\":\"offline\"}"
 *     3. Subscribe to the command topic:
 *          AT+QMTSUB=0,1,"smartwagon/v1/<wgn>/<gw>/dn/cmd",1
 *     4. Publish birth (retained) to the status topic: {"st":"online"}
 *
 *   Then implement ec200_mqtt_poll_cmd() by scanning ec_rb for the URC:
 *     +QMTRECV: <idx>,<msgID>,"<topic>",<len>,"<payload>"
 *   extract <payload> into `out`. (Reuse the ec_wait()/ring-buffer plumbing
 *   already in ec200.c; that is why the real body lives there.)
 *
 * DONE: ec200_mqtt_up() now sets a persistent session + last-will and
 * subscribes to dn/cmd; ec200_mqtt_poll_cmd() parses the +QMTRECV URC. Both
 * live in ec200.c (which owns the modem UART + ec_rb), so this file no longer
 * defines the function - it is kept only as the integration note above.
 */
