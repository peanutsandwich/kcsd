/**
 * Daemon that forwards requests from KCS over the DBUS IPMI Interface
 * 
 * This daemon is based on btbridged from https://github.com/openbmc/btbridge
 * but since KCS is a simpler protocol, it has been somewhat simplified. The
 * main simplification comes from the fact that KCS is a sequential protocol
 * where both sides have to agree on the state of the transfer, so there is no
 * need to queue messages for KCS since they should not arrive out of sequence.
 * 
 * The timer is still used, as we want to abort 'stuck' commands after an expiry
 * time. Messages that are received out of order (i.e. requests while awaiting a
 * resp, and resps while awaiting a request) are discarded.
 *
 * Copyright (C) 2018 SerialTek UK Ltd.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <time.h>
#include <errno.h>
#include <libgen.h>
#include <unistd.h>
#include <fcntl.h>

#include <systemd/sd-bus.h>

/*******************************************************************************
 * Constants and Types
 ******************************************************************************/
/** Indicies of the file descriptors to watch for events on */
#define SD_BUS_FD                   0
#define KCS_DEV_FD                  1
#define TIMER_FD                    2
/** Number of file descriptors to watch - must keep up to date with the list */
#define FD_COUNT                    3

/** Maximum length of a KCS message */
#define KCS_MAX_MESSAGE_LEN         128

/** Interface name to register our methods and signals with */
#define INTERFACE_NAME              "org.openbmc.HostIpmi"

/** Format for the bus name. We use the basename of the KCS driver to 
 *  distinuish between instances of the daemon that are spun up against 
 *  different drivers (since ASPEED chips have 4 KCS interfaces)  */
#define BUS_NAME_FMT                "org.openbmc.HostIpmi.%s"
#define BUS_NAME_LEN                32

/** Format for the object name. We use the basename of the KCS driver to 
 *  distinuish between instances of the daemon that are spun up against 
 *  different drivers (since ASPEED chips have 4 KCS interfaces) */
#define OBJECT_NAME_FMT             "/org/openbmc/HostIpmi/%s"
#define OBJECT_NAME_LEN             32

/** Offsets in the IPMI message format */
#define NETFN_LUN_OFFSET            0
#define CMD_OFFSET                  1
#define DATA_OFFSET_REQ             2
/** Responses contain an additional byte for the completion code */
#define CC_OFFSET                   2
#define DATA_OFFSET_RESP            3

/** Shifts and masks for extracting the LUN and NetFn */
#define NETFN_SHIFT                 2
#define LUN_MASK                    0x3

/** Completion code we use when a command times out */
#define IPMI_CC_CANNOT_PROVIDE_RESP 0xce

/** Timeout in seconds used for KCS responses */
#define KCS_TIMEOUT_SECONDS         5

/** Structure containing the parameters of a message */
typedef struct _MsgEntry_s
{
    uint8_t         lun;
    uint8_t         netfn;
    uint8_t         cmd;
}MsgEntry_t;

/** Structure containing the context for the daemon */
typedef struct _KcsdContext_s
{
    /** The object name to use in dbus for this instance */
    char            objName[OBJECT_NAME_LEN];
    /** The bus name to use for this instance - a unique busname gives the ipmi
     *  daemon the 'address' to respond to a message on */
    char            busName[BUS_NAME_LEN];

    /** The file descriptors to poll */
    struct pollfd   fds[FD_COUNT];

    /**< Pointer to the system dbus */
    struct sd_bus*  pBus;

    /** Tracking variable for a pending message so that if it times out we can
     *  send a response using the correct LUN, netfn and command to indicate to
     *  the host that we timed out waiting for a response */
    MsgEntry_t      pendingMsg;

    /** Flag to indicate whether we are awaiting a response */
    int             awaitingResponse;
}KcsdContext_t;

#ifdef DEBUG_KCSD
#define DEBUG_PRINT(__fmt, ...) \
    do { printf(__fmt, ## __VA_ARGS__); } while (0)
#else
#define DEBUG_PRINT(__fmt, ...)
#endif

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static int handle_dbus_events(
    KcsdContext_t*  pContext);

static int handle_kcs_events(
    KcsdContext_t*  pContext);

static int handle_timer_events(
    KcsdContext_t*  pContext);

static int kcsd_method_send_message(
    sd_bus_message* pMsg,
    void*           pUserData,
    sd_bus_error*   pRetError);

/*******************************************************************************
 * Exported Variables
 ******************************************************************************/

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
/** Methods and signals registered by this dbus service */
static const sd_bus_vtable kcsd_vtable[] = {
    SD_BUS_VTABLE_START(0),
    SD_BUS_METHOD("sendMessage", "yyyyyay", "x",
                  &kcsd_method_send_message, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_SIGNAL("ReceivedMessage", "yyyyay", 0),
    SD_BUS_VTABLE_END
};

/*******************************************************************************
 * Exported Functions
 ******************************************************************************/
int main(int argc, char* argv[])
{
    KcsdContext_t context;
    int ret = 0;
    const char* driverPath = NULL;
    char* kcsDevname = NULL;

    if(argc <= 1)
    {
        /* Need the path to the KCS device driver */
        printf("Please specify a path to the KCS driver instance\n");
        return -EINVAL;
    }
    else
    {
        driverPath = argv[1];
    }

    /* Extract the device name from the driver path. IPMI KCS drivers are of the 
     * form /dev/ipmi-kcsX, so we want to extract the 'kcsX' bit for use in the
     * object name and bus name */
    kcsDevname = strstr(driverPath, "-kcs");
    if(!kcsDevname)
    {
        /* Invalid path */
        printf("Pathname \"%s\" is not a valid driver path\n", driverPath);
        return -EINVAL;
    }

    snprintf(context.objName, sizeof(context.objName),
             OBJECT_NAME_FMT, kcsDevname + 1);

    snprintf(context.busName, sizeof(context.busName),
             BUS_NAME_FMT, kcsDevname + 1);

    DEBUG_PRINT("Getting system bus\n");
    ret = sd_bus_default_system(&context.pBus);
    if(ret < 0)
    {
        printf("Failed to connect to system bus (ret: %d, errno: %d)\n",
               ret, errno);
        return -1;
    }

    DEBUG_PRINT("Registering message and signal handlers\n");
    ret = sd_bus_add_object_vtable(context.pBus,
                                   NULL,
                                   context.objName,
                                   INTERFACE_NAME,
                                   kcsd_vtable,
                                   &context);
    if(ret < 0)
    {
        printf("Failed to register methods and signals (ret: %d, errno: %d)\n",
               ret, errno);
        sd_bus_unref(context.pBus);
        return -1;
    }

    ret = sd_bus_request_name(context.pBus,
                              context.busName,
                              SD_BUS_NAME_ALLOW_REPLACEMENT |
                              SD_BUS_NAME_REPLACE_EXISTING);
    if(ret < 0)
    {
        printf("Failed to acquire name %s (ret: %d, errno: %d)\n",
               context.busName, ret, errno);
        sd_bus_unref(context.pBus);
        return -1;
    }

    DEBUG_PRINT("Getting dbus fd\n");
    context.fds[SD_BUS_FD].fd = sd_bus_get_fd(context.pBus);
    if(context.fds[SD_BUS_FD].fd < 0)
    {
        printf("Failed to get dbus file descriptor (ret: %d, errno: %d)",
               context.fds[SD_BUS_FD].fd, errno);
        sd_bus_unref(context.pBus);
        return -1;
    }

    DEBUG_PRINT("Opening driver @ %s\n",
                driverPath);
    context.fds[KCS_DEV_FD].fd = open(driverPath, O_RDWR);
    if(context.fds[KCS_DEV_FD].fd < 0)
    {
        printf("Failed to open device @ %s (ret: %d, errno: %d)\n",
               driverPath, context.fds[KCS_DEV_FD].fd, errno);
        sd_bus_unref(context.pBus);
        return -errno;
    }

    DEBUG_PRINT("Creating timer fd\n");
    context.fds[TIMER_FD].fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if(context.fds[TIMER_FD].fd < 0)
    {
        printf("Failed to create timer fd (ret: %d, errno: %d)\n", ret, errno);
        return -errno;
    }

    context.fds[SD_BUS_FD].events = POLLIN;
    context.fds[KCS_DEV_FD].events = POLLIN;
    context.fds[TIMER_FD].events = POLLIN;

    DEBUG_PRINT("Polling...\n");

    while(1)
    {
        ret = poll(context.fds, FD_COUNT, -1);
        if(ret < 0)
        {
            printf("Error from poll (ret: %d, errno: %d)\n", ret, errno);
            break;
        }
        else
        {
            ret = handle_dbus_events(&context);
            if(ret < 0)
            {
                printf("Error handling dbus events (ret: %d, errno: %d)\n",
                       ret, errno);
                break;
            }

            ret = handle_kcs_events(&context);
            if(ret < 0)
            {
                printf("Error handling kcs events (ret: %d, errno: %d)\n",
                       ret, errno);
                break;
            }

            ret = handle_timer_events(&context);
            if(ret < 0)
            {
                printf("Error handling timer events (ret: %d, errno: %d)\n",
                       ret, errno);
                break;
            }
        }
    }

    close(context.fds[KCS_DEV_FD].fd);
    close(context.fds[TIMER_FD].fd);
    sd_bus_unref(context.pBus);

    return ret;
}

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
/**
 * Send a response on the KCS driver
 * 
 * @param[in]       pContext    The daemon's context
 * @param[in]       pMsg        The message parameters to send
 * @param[in]       cc          The completion code for the message
 * @param[in]       pData       The data to send with the response
 * @param[in]       dataLen     The length of the data buffer
 * 
 * @return  The number of bytes written, or a negative value on failure
 */
static int kcs_send_response(
    KcsdContext_t*  pContext,
    MsgEntry_t*     pMsg,
    uint8_t         cc,
    uint8_t*        pData,
    size_t          dataLen)
{
    int ret = 0;

    if(dataLen > (KCS_MAX_MESSAGE_LEN - DATA_OFFSET_RESP))
    {
        printf("Data length is too long\n");
        ret = -EINVAL;
    }
    else
    {
        uint8_t messageData[KCS_MAX_MESSAGE_LEN] = {0};

        /* Populate the message buffer */
        messageData[NETFN_LUN_OFFSET] = (pMsg->netfn << NETFN_SHIFT) |
                                        (pMsg->lun & LUN_MASK);
        messageData[CMD_OFFSET] = pMsg->cmd;
        messageData[CC_OFFSET] = cc;
        memcpy(&messageData[DATA_OFFSET_RESP], pData, dataLen);

#ifdef DEBUG_KCSD_RESPONSES
        DEBUG_PRINT("Resp --> LUN:%d, Netfn:0x%02x, Cmd:0x%02x, CC:0x%02x, Data:",
                    pMsg->lun, pMsg->netfn, pMsg->cmd, cc);
        {
            uint8_t dataByte = 0;
            for(dataByte = 0; dataByte < dataLen; ++dataByte)
            {
                DEBUG_PRINT("%02x ", pData[dataByte]);
            }
        }
        DEBUG_PRINT("\n");
#endif

        ret = write(pContext->fds[KCS_DEV_FD].fd,
                    messageData,
                    dataLen + DATA_OFFSET_RESP);
        if(ret < 0)
        {
            printf("Failed to write to driver (ret: %d, errno: %d)\n",
                   ret, errno);
            ret = -errno;
        }
        else if(ret < DATA_OFFSET_RESP)
        {
            /* Not enough data */
            printf("Not enough data sent\n");
            ret = -EINVAL;
        }
    }

    return ret;
}

/**
 * Send a response on the KCS driver
 * 
 * @param[in]       pContext    The daemon's context
 * @param[in]       pMsg        The message parameters to send
 * @param[in]       pData       The data to send with the response
 * @param[in]       dataLen     The length of the data buffer
 * 
 * @return  A positive value if successful, or a negative value on failure
 */
static int send_received_message_signal(
    KcsdContext_t*  pContext,
    MsgEntry_t*     pMsg,
    uint8_t*        pData,
    uint8_t         dataLen)
{
    sd_bus_message* pSignal = NULL;
    int ret = 0;

    ret = sd_bus_message_new_signal(pContext->pBus,
                                    &pSignal,
                                    pContext->objName,
                                    INTERFACE_NAME,
                                    "ReceivedMessage");
    if(ret < 0)
    {
        printf("Failed to create ReceivedMessage signal (ret: %d, errno: %d)\n",
               ret, errno);
        return ret;
    }

    ret = sd_bus_message_append(pSignal, "yyyy",
                                0,
                                pMsg->netfn,
                                pMsg->lun,
                                pMsg->cmd);
    if(ret < 0)
    {
        printf("Failed to append parameters to signal (ret: %d, errno: %d)\n",
               ret, errno);
        sd_bus_message_unref(pSignal);
        return ret;
    }

    ret = sd_bus_message_append_array(pSignal, 'y',
                                      pData,
                                      dataLen);
    if(ret < 0)
    {
        printf("Failed to append data to signal (ret: %d, errno: %d)\n",
               ret, errno);
        sd_bus_message_unref(pSignal);
        return ret;
    }

    ret = sd_bus_send(pContext->pBus, pSignal, NULL);
    if(ret < 0)
    {
        printf("Failed to emit signal (ret: %d, errno: %d)\n",
               ret, errno);
    }

    sd_bus_message_unref(pSignal);

    return ret;
}

/**
 * Handle events on the dbus file descriptor
 * 
 * @param[in]       pContext    The daemon's context
 * 
 * @return  A positive value on success, negative on failure
 */
static int handle_dbus_events(
    KcsdContext_t*  pContext)
{
    int ret = 0;

    if(pContext->fds[SD_BUS_FD].revents & POLLIN)
    {
        ret = sd_bus_process(pContext->pBus, NULL);
        if(ret < 0)
        {
            printf("Failed to process dbus events (ret: %d)\n", ret);
        }
    }

    return ret;
}

/**
 * Handle events on the KCS driver file descriptor
 * 
 * @param[in]       pContext    The daemon's context
 * 
 * @return  A positive value on success, negative on failure
 */
static int handle_kcs_events(
    KcsdContext_t*  pContext)
{
    int ret = 0;

    if(pContext->fds[KCS_DEV_FD].revents & POLLIN)
    {
        /* We've received data on the driver */
        struct itimerspec ts;
        uint8_t messageData[KCS_MAX_MESSAGE_LEN] = {0};
        uint8_t dataLen = 0;

        ret = read(pContext->fds[KCS_DEV_FD].fd,
                   messageData,
                   sizeof(messageData));
        if(ret < 0)
        {
            printf("Failed to read data from KCS (ret: %d, errno: %d)\n",
                        ret, errno);
            return ret;
        }
        if(ret < DATA_OFFSET_REQ)
        {
            printf("Not enough data read for a KCS message (ret: %d)\n",
                   ret);
            return -1;
        }

        dataLen = ret;

        if(pContext->awaitingResponse)
        {
            DEBUG_PRINT("Received KCS message while awaiting response. Discarding\n");
        }
        else
        {
            pContext->awaitingResponse = 1;
            pContext->pendingMsg.lun = messageData[NETFN_LUN_OFFSET] & LUN_MASK;
            pContext->pendingMsg.netfn = messageData[NETFN_LUN_OFFSET] >> NETFN_SHIFT;
            pContext->pendingMsg.cmd = messageData[CMD_OFFSET];

#ifdef DEBUG_KCSD_REQUESTS
            DEBUG_PRINT("Req  <-- LUN:%d, Netfn:0x%02x, Cmd:0x%02x, Data:",
                        pContext->pendingMsg.lun,
                        pContext->pendingMsg.netfn,
                        pContext->pendingMsg.cmd);
            {
                uint8_t dataByte = 0;
                for(dataByte = 0; dataByte < (dataLen - DATA_OFFSET_REQ); ++dataByte)
                {
                    DEBUG_PRINT("%02x ", messageData[DATA_OFFSET_REQ + dataByte]);
                }
            }
            DEBUG_PRINT("\n");
#endif

            /* Set up the timer. We do this before sending the signal to avoid
             * a race condition with the response */
            ts.it_interval.tv_sec = 0;
            ts.it_interval.tv_nsec = 0;
            ts.it_value.tv_sec = KCS_TIMEOUT_SECONDS;
            ts.it_value.tv_nsec = 0;
            ret = timerfd_settime(pContext->fds[TIMER_FD].fd, 0, &ts, NULL);
            if(ret < 0)
            {
                printf("Failed to set timer (ret: %d, errno: %d)\n",
                       ret, errno);
            }

            ret = send_received_message_signal(pContext,
                                               &pContext->pendingMsg,
                                               messageData + DATA_OFFSET_REQ,
                                               dataLen - DATA_OFFSET_REQ);
            if(ret < 0)
            {
                printf("Failed to send ReceivedMessage signal (ret: %d)\n", ret);
            }
        }
    }

    return ret;
}

/**
 * Handle events on the timer file descriptor
 * 
 * @param[in]       pContext    The daemon's context
 * 
 * @return  A positive value on success, negative on failure
 */
static int handle_timer_events(
    KcsdContext_t*  pContext)
{
    int ret = 0;

    if(pContext->fds[TIMER_FD].revents & POLLIN)
    {
        if(!pContext->awaitingResponse)
        {
            /* Strange, we got a timeout but weren't expecting a response */
            DEBUG_PRINT("Timeout but no pending message\n");
        }
        else
        {
            struct itimerspec ts;

            /* Add one to the netfn - response netfn is always request netfn + 1 */
            pContext->pendingMsg.netfn += 1;

            /* Clear the timer */
            ts.it_interval.tv_sec = 0;
            ts.it_interval.tv_nsec = 0;
            ts.it_value.tv_sec = 0;
            ts.it_value.tv_nsec = 0;
            ret = timerfd_settime(pContext->fds[TIMER_FD].fd,
                                  TFD_TIMER_ABSTIME,
                                  &ts,
                                  NULL);
            if(ret < 0)
            {
                DEBUG_PRINT("Failed to clear timer\n");
            }

            DEBUG_PRINT("Timing out message\n");
            ret = kcs_send_response(pContext,
                                    &pContext->pendingMsg,
                                    IPMI_CC_CANNOT_PROVIDE_RESP,
                                    NULL,
                                    0);
            if(ret < 0)
            {
                printf("Failed to send timeout message (ret: %d, errno: %d)\n",
                       ret, errno);
            }

            pContext->awaitingResponse = 0;
        }
    }

    return ret;
}

/**
 * Dbus handler for the sendMessage method
 * 
 * @param[in]       pMsg        The received message
 * @param[in]       pUserData   Pointer to the context
 * @param[in]       pRetError   The object to populate when an error occurs
 * 
 * @return  Positive value if successful, negative value on failure
 */
static int kcsd_method_send_message(
    sd_bus_message* pMsg,
    void*           pUserData,
    sd_bus_error*   pRetError)
{
    int ret = 1;
    KcsdContext_t* pContext = (KcsdContext_t*)pUserData;
    sd_bus_message* pRespMsg = NULL;

    ret = sd_bus_message_new_method_return(pMsg, &pRespMsg);
    if(ret < 0)
    {
        printf("Failed to create method response (ret: %d)\n", ret);
        return ret;
    }

    if(!pContext->awaitingResponse)
    {
        /* We aren't expecting a response at this time */
        printf("Response message received when in wrong state. Discarding\n");
        ret = -EBUSY;
    }
    else
    {
        uint8_t seq = 0;
        MsgEntry_t resp;
        uint8_t cc = 0;
        uint8_t* pData = NULL;
        size_t dataLen = 0;
        struct itimerspec ts;

        if(!pContext)
        {
            sd_bus_error_set_const(pRetError,
                                   "org.openbmc.error",
                                   "Internal error");

            ret = -EINVAL;
            goto done;
        }

        pContext->awaitingResponse = 0;

        ret = sd_bus_message_read(pMsg, "yyyyy", &seq,
                                                 &resp.netfn,
                                                 &resp.lun,
                                                 &resp.cmd,
                                                 &cc);
        if(ret < 0)
        {
            printf("Failed to read message parameters (ret: %d)\n", ret);
            goto done;
        }

        ret = sd_bus_message_read_array(pMsg, 'y', (const void**)&pData,
                                                   &dataLen);
        if(ret < 0)
        {
            printf("Failed to read message data (ret: %d)\n", ret);
            ret = -EINVAL;
            goto done;
        }

        /* Clear the timer */
        ts.it_interval.tv_sec = 0;
        ts.it_interval.tv_nsec = 0;
        ts.it_value.tv_sec = 0;
        ts.it_value.tv_nsec = 0;
        ret = timerfd_settime(pContext->fds[TIMER_FD].fd,
                              TFD_TIMER_ABSTIME,
                              &ts,
                              NULL);
        if(ret < 0)
        {
            DEBUG_PRINT("Failed to clear timer\n");
        }

        ret = kcs_send_response(pContext, &resp, cc, pData, dataLen);
    }

done:
    ret = sd_bus_message_append(pRespMsg, "x", ret);
    if(ret < 0)
    {
        printf("Failed to add result to method (ret: %d)\n", ret);
    }

    ret = sd_bus_send(pContext->pBus, pRespMsg, NULL);
    if(ret < 0)
    {
        printf("Failed to send response (ret: %d)\n", ret);
    }

    return ret;
}
