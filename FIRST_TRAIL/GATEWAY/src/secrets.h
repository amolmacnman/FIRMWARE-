/*
 * BROKER CREDENTIALS - NOT IN VERSION CONTROL.
 *
 * This file is listed in .gitignore and must never be committed. It holds the
 * only values in the build that are genuinely secret; everything else about the
 * broker (topic root, QoS, retain policy) is in app_config.h and is public.
 *
 * If this file is absent the build still works: app_config.h detects it with
 * __has_include() and falls back to the anonymous public broker, so a fresh
 * clone compiles and runs without any credential.
 *
 * To set up a new working copy, copy secrets.h.example to secrets.h and fill in
 * the real values.
 *
 * NOTE ON EXPOSURE: MQTT sends the username and password as PLAINTEXT inside
 * the CONNECT packet. On a 1883 listener anyone observing the path reads them
 * off the wire, so keeping them out of git limits where they leak - it does
 * NOT make the link secure.
 *
 * ON TLS IN THIS TREE: this text used to say the tree had no TLS support at
 * all. That was copied from the DK bench tree and is WRONG here - this IS the
 * 52-pin production tree, and ec200.c does carry the QSSLCFG block. The switch
 * is MQTT_TLS in app_config.h.
 *
 * It is currently 0, which is a KNOWN DEVIATION FROM RDSO s3.8, not a missing
 * feature: port 8883 on the JNV host is closed (verified), so there is nothing
 * to negotiate against. Turning MQTT_TLS on needs a broker-side 8883 listener
 * AND the CA certificate written to the modem's filesystem as ca.pem.
 */
#ifndef SECRETS_H
#define SECRETS_H

/* ===========================================================================
 * WHICH BROKER TO BUILD FOR - change this ONE line, then rebuild.
 *
 *   1 = JNV      95.216.167.38   authenticated (jnv_mqtt)
 *   0 = MACNMAN  mqtt.macnman.com  anonymous, no credentials
 *
 * Both are plain MQTT on port 1883. Neither currently has a TLS listener:
 * port 8883 on the JNV host is closed (verified). In the 52-pin tree that
 * means MQTT_TLS must stay 0; this tree has no TLS support to enable.
 * ======================================================================== */
#define USE_JNV_BROKER   1

#if USE_JNV_BROKER

#define MQTT_HOST      "95.216.167.38"
#define MQTT_USERNAME  "jnv_mqtt"
#define MQTT_PASSWORD  "JnVMQ@1507!"

#else   /* macnman - anonymous */

/* An empty MQTT_USERNAME makes ec200.c issue the 2-argument AT+QMTCONN and
 * send no credentials at all, which is what this broker expects. */
#define MQTT_HOST      "mqtt.macnman.com"
#define MQTT_USERNAME  ""
#define MQTT_PASSWORD  ""

#endif  /* USE_JNV_BROKER */

#endif /* SECRETS_H */
