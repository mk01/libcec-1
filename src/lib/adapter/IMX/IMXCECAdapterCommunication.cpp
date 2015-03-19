/*
 * This file is part of the libCEC(R) library.
 *
 * libCEC(R) is Copyright (C) 2011-2013 Pulse-Eight Limited.  All rights reserved.
 * libCEC(R) is an original work, containing original code.
 *
 * libCEC(R) is a trademark of Pulse-Eight Limited.
 * 
 * IMX adpater port is Copyright (C) 2013 by Stephan Rafin
 *                     Copyright (C) 2014 by Matus Kral
 * 
 * You can redistribute this file and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *
 */

#include "env.h"

#if defined(HAVE_IMX_API)
#include "IMXCECAdapterCommunication.h"

#include "lib/CECTypeUtils.h"
#include "lib/LibCEC.h"
#include "lib/platform/sockets/cdevsocket.h"
#include "lib/platform/util/StdString.h"
#include "platform/util/util.h"

using namespace std;
using namespace CEC;
using namespace PLATFORM;

#include "IMXCECAdapterMessageQueue.h"

#define LIB_CEC m_callback->GetLib()

// these are defined in nxp private header file
#define CEC_MSG_SUCCESS                 0x00	/*Message transmisson Succeed*/
#define CEC_CSP_OFF_STATE               0x80	/*CSP in Off State*/
#define CEC_BAD_REQ_SERVICE             0x81	/*Bad .req service*/
#define CEC_MSG_FAIL_UNABLE_TO_ACCESS	0x82	/*Message transmisson failed: Unable to access CEC line*/
#define CEC_MSG_FAIL_ARBITRATION_ERROR	0x83	/*Message transmisson failed: Arbitration error*/
#define CEC_MSG_FAIL_BIT_TIMMING_ERROR	0x84	/*Message transmisson failed: Bit timming error*/
#define CEC_MSG_FAIL_DEST_NOT_ACK       0x85	/*Message transmisson failed: Destination Address not aknowledged*/
#define CEC_MSG_FAIL_DATA_NOT_ACK       0x86	/*Message transmisson failed: Databyte not acknowledged*/

CIMXCECAdapterCommunication::CIMXCECAdapterCommunication(IAdapterCommunicationCallback *callback) :
    IAdapterCommunication(callback),
    m_PAReporter(NULL)
{
  CLockObject lock(m_mutex);

  m_iNextMessage = 0;
  m_logicalAddress = CECDEVICE_UNKNOWN;
  m_bLogicalAddressRegistered = false;
  m_bInitialised = false;
  m_dev = new CCDevSocket(CEC_IMX_PATH);
  m_physicalAddress = -1;
}

CIMXCECAdapterCommunication::~CIMXCECAdapterCommunication(void)
{
  Close();
  DELETE_AND_NULL(m_PAReporter);
  delete m_dev;
  m_dev = 0;
}

bool CIMXCECAdapterCommunication::IsOpen(void)
{
  return IsInitialised() && m_dev->IsOpen();
}

bool CIMXCECAdapterCommunication::Open(uint32_t iTimeoutMs, bool UNUSED(bSkipChecks), bool bStartListening)
{
  if (m_dev->Open(iTimeoutMs))
  {
    if (!bStartListening || CreateThread()) {
      if (m_dev->Ioctl(HDMICEC_IOC_STARTDEVICE, NULL) == 0) {
        m_bInitialised = true;
        RegisterLogicalAddress(CECDEVICE_BROADCAST);
        return true;
      }
      LIB_CEC->AddLog(CEC_LOG_ERROR, "%s: Unable to start device\n", __func__);
    }
    m_dev->Close();
  }

  return false;
}


void CIMXCECAdapterCommunication::Close(void)
{
  StopThread(-1);
  if (m_bInitialised)
  {
    m_bInitialised = false;
    UnregisterLogicalAddress();

    if (m_dev->Ioctl(HDMICEC_IOC_STOPDEVICE, NULL) != 0)
    {
      LIB_CEC->AddLog(CEC_LOG_ERROR, "%s: Unable to stop device\n", __func__);
    }
  }
  m_dev->Close();
}


std::string CIMXCECAdapterCommunication::GetError(void) const
{
  std::string strError(m_strError);
  return strError;
}


cec_adapter_message_state CIMXCECAdapterCommunication::Write(
  const cec_command &data, bool &bRetry, uint8_t iLineTimeout, bool UNUSED(bIsReply))
{
  unsigned char message[MAX_MESSAGE_LEN];
  CIMXCECAdapterMessageQueueEntry *entry;
  int msg_len = 1;
  cec_adapter_message_state rc = ADAPTER_MESSAGE_STATE_ERROR;

  bRetry = true;
  if ((size_t)data.parameters.size + data.opcode_set + 1 > sizeof(message))
  {
    LIB_CEC->AddLog(CEC_LOG_ERROR, "%s: data size too large !", __func__);
    bRetry = false;
    return rc;
  }

  message[0] = (data.initiator << 4) | (data.destination & 0x0f);
  if (data.opcode_set)
  {
    message[1] = data.opcode;
    msg_len++;
    memcpy(&message[2], data.parameters.data, data.parameters.size);
    msg_len+=data.parameters.size;
  }

  entry = new CIMXCECAdapterMessageQueueEntry(message[0], data.opcode);
  m_messageMutex.Lock();
  uint32_t msgKey = ++m_iNextMessage;
  m_messages.insert(make_pair(msgKey, entry));
  m_messageMutex.Unlock();

  if (m_dev->Write(message, msg_len) > 0)
  { 
    if (entry->Wait(data.transmit_timeout ? data.transmit_timeout : iLineTimeout *1000))
    {
      int status = entry->Result();

      if (status == MESSAGE_TYPE_NOACK)
        rc = ADAPTER_MESSAGE_STATE_SENT_NOT_ACKED;
      else if (status == MESSAGE_TYPE_SEND_SUCCESS)
        rc = ADAPTER_MESSAGE_STATE_SENT_ACKED;

      bRetry = false;
    }
    else
    {
      rc = ADAPTER_MESSAGE_STATE_WAITING_TO_BE_SENT;
#ifdef CEC_DEBUGGING
      LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s: command timed out !", __func__);
#endif
    }
  }
  else
  {
    Sleep(CEC_DEFAULT_TRANSMIT_RETRY_WAIT);
#ifdef CEC_DEBUGGING
    LIB_CEC->AddLog(CEC_LOG_WARNING, "%s: write failed !", __func__);
#endif
  }

  m_messageMutex.Lock();
  m_messages.erase(msgKey);
  m_messageMutex.Unlock();

  delete entry;

  return rc;
}


uint16_t CIMXCECAdapterCommunication::GetFirmwareVersion(void)
{
  /* FIXME add ioctl ? */
  return 0;
}


cec_vendor_id CIMXCECAdapterCommunication::GetVendorId(void)
{
  return CEC_VENDOR_UNKNOWN;
}


uint16_t CIMXCECAdapterCommunication::GetPhysicalAddress(void)
{
  uint8_t phy_addr[4];
  uint16_t pa_tmp;

  if (m_dev->Ioctl(HDMICEC_IOC_GETPHYADDRESS, &phy_addr) != 0)
  {
    LIB_CEC->AddLog(CEC_LOG_ERROR, "%s: HDMICEC_IOC_GETPHYADDRESS failed !", __func__);
    return CEC_INVALID_PHYSICAL_ADDRESS; 
  }

  if ((pa_tmp = ((phy_addr[0] << 4 | phy_addr[1]) << 8) | (phy_addr[2] << 4 | phy_addr[3])))
    m_physicalAddress = pa_tmp;

  return m_physicalAddress;
}


cec_logical_addresses CIMXCECAdapterCommunication::GetLogicalAddresses(void)
{
  cec_logical_addresses addresses;
  addresses.Clear();

  CLockObject lock(m_mutex);
  if (m_bLogicalAddressRegistered)
    addresses.Set(m_logicalAddress);

  return addresses;
}

void CIMXCECAdapterCommunication::HandleLogicalAddressLost(cec_logical_address UNUSED(oldAddress))
{
  UnregisterLogicalAddress();
}

bool CIMXCECAdapterCommunication::UnregisterLogicalAddress(void)
{
  {
    CLockObject lock(m_mutex);
    if (!m_bLogicalAddressRegistered)
      return true;
  }

#ifdef CEC_DEBUGGING
  LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s - releasing previous logical address", __func__);
#endif
  return RegisterLogicalAddress(CECDEVICE_BROADCAST);
}

bool CIMXCECAdapterCommunication::RegisterLogicalAddress(const cec_logical_address address)
{
  {
    CLockObject lock(m_mutex);
    if ((m_logicalAddress == address && m_bLogicalAddressRegistered) ||
        (m_logicalAddress == address && address == CECDEVICE_BROADCAST))
    {
      return true;
    }
  }

#ifdef CEC_DEBUGGING
  LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s: %x to %x", __func__, m_logicalAddress, address);
#endif

  if (m_dev->Ioctl(HDMICEC_IOC_SETLOGICALADDRESS, (void *)address) != 0)
  {
    LIB_CEC->AddLog(CEC_LOG_ERROR, "%s: HDMICEC_IOC_SETLOGICALADDRESS failed !", __func__);
    return false;
  }

  CLockObject lock(m_mutex);

  m_logicalAddress = address;
  m_bLogicalAddressRegistered = (address != CECDEVICE_BROADCAST) ? true : false;
  return true;
}

bool CIMXCECAdapterCommunication::SetLogicalAddresses(const cec_logical_addresses &addresses)
{
  int log_addr = addresses.primary;

  return RegisterLogicalAddress((cec_logical_address)log_addr);
}


void *CIMXCECAdapterCommunication::Process(void)
{
  bool bHandled;
  hdmi_cec_event event;
  int ret;

  cec_logical_address initiator, destination;

  while (!IsStopped())
  {
    if (IsInitialised() && (ret = m_dev->Read((char *)&event, sizeof(event), 5000)) > 0)
    {

      initiator = cec_logical_address(event.msg[0] >> 4);
      destination = cec_logical_address(event.msg[0] & 0x0f);

        if (event.event_type == MESSAGE_TYPE_RECEIVE_SUCCESS)
        {
            cec_command cmd;

            cec_command::Format(
                cmd, initiator, destination,
                ( event.msg_len > 1 ) ? cec_opcode(event.msg[1]) : CEC_OPCODE_NONE);

            for( uint8_t i = 2; i < event.msg_len; i++ )
                cmd.parameters.PushBack(event.msg[i]);

            if (!IsStopped()) {
              m_callback->OnCommandReceived(cmd);
            }
        }
        else if (event.event_type == MESSAGE_TYPE_SEND_SUCCESS 
                || event.event_type == MESSAGE_TYPE_NOACK)
        {
            bHandled = false;

            m_messageMutex.Lock();
            for (map<uint32_t, CIMXCECAdapterMessageQueueEntry *>::iterator it = m_messages.begin();
              !bHandled && it != m_messages.end(); it++)
              {
                bHandled = it->second->Received(event.event_type, event.msg[0], (cec_opcode)event.msg[1]);
              }
            m_messageMutex.Unlock();

            if (!bHandled)
              LIB_CEC->AddLog(CEC_LOG_WARNING, "%s: response not matched !", __func__);
        }
        else if (event.event_type == MESSAGE_TYPE_DISCONNECTED)
        {
            /* HDMI Hotplug event - disconnect */
        }
        else if (event.event_type == MESSAGE_TYPE_CONNECTED && m_physicalAddress != 0xffff)
        {
            /* HDMI Hotplug event - connect */
            uint16_t oldAddress = m_physicalAddress;

            if (oldAddress != GetPhysicalAddress()) {
              if (m_PAReporter)
                while (m_PAReporter->IsRunning()) Sleep(5);
              delete m_PAReporter;

              m_PAReporter = new CCECPAChangedReporter(m_callback, m_physicalAddress);
              m_PAReporter->CreateThread();
            }
#ifdef CEC_DEBUGGING
            LIB_CEC->AddLog(CEC_LOG_DEBUG, "%s: plugin event received", __func__);
#endif
        }
        else
            LIB_CEC->AddLog(CEC_LOG_WARNING, "%s: unhandled response received %d!", __func__, event.event_type);
    }
  }

  return 0;
}

CCECPAChangedReporter::CCECPAChangedReporter(IAdapterCommunicationCallback *callback, uint16_t newPA) :
    m_callback(callback),
    m_newPA(newPA)
{
}

void* CCECPAChangedReporter::Process(void)
{
  m_callback->HandlePhysicalAddressChanged(m_newPA);
  return NULL;
}

#endif	// HAVE_IMX_API
