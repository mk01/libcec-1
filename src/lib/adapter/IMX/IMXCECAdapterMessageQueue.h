#pragma once
/*
 * This file is part of the libCEC(R) library.
 *
 * libCEC(R) is Copyright (C) 2011-2013 Pulse-Eight Limited.  All rights reserved.
 * libCEC(R) is an original work, containing original code.
 *
 * libCEC(R) is a trademark of Pulse-Eight Limited.
 *
 * This program is dual-licensed; you can redistribute it and/or modify
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
 * Alternatively, you can license this library under a commercial license,
 * please contact Pulse-Eight Licensing for more information.
 *
 * For more information contact:
 * Pulse-Eight Licensing       <license@pulse-eight.com>
 *     http://www.pulse-eight.com/
 *     http://www.pulse-eight.net/
 */

#include "lib/platform/threads/mutex.h"

namespace CEC
{
  using namespace PLATFORM;
  
  class CIMXCECAdapterMessageQueueEntry
  {
  public:
    CIMXCECAdapterMessageQueueEntry(uint8_t addrs, cec_opcode opcode)
       : m_bWaiting(true), m_retval((uint32_t)-1), m_bSucceeded(false)
    {
      m_opcode = opcode;
      m_addrs = addrs; 
    }
    
    virtual ~CIMXCECAdapterMessageQueueEntry(void) {}

    /*!
     * @brief Query result from worker thread
     */
    uint32_t Result() const
    {
      return m_retval;
    }
    
    /*!
     * @brief Signal waiting threads
     */
    void Broadcast(void)
    {
      CLockObject lock(m_mutex);
      m_condition.Broadcast();
    }

    /*!
     * @brief Signal waiting thread(s) when message matches this entry
     */
    bool Received(int response, uint8_t addrs, cec_opcode opcode)
    {
      CLockObject lock(m_mutex);

      if (!(m_addrs == addrs && m_opcode == opcode))
        return false;

      m_retval = response;
      m_bSucceeded = true;
      m_condition.Signal();
      return true;
    }

    /*!
     * @brief Wait for a response to this command.
     * @param iTimeout The timeout to use while waiting.
     * @return True when a response was received before the timeout passed, false otherwise.
     */
    bool Wait(uint32_t iTimeout)
    {
      CLockObject lock(m_mutex);
      
      bool bReturn = m_bSucceeded ? true : m_condition.Wait(m_mutex, m_bSucceeded, iTimeout);
      m_bWaiting = false;
      return bReturn;
    }

    /*!
     * @return True while a thread is waiting for a signal or isn't waiting yet, false otherwise.
     */
    bool IsWaiting(void)
    {
      CLockObject lock(m_mutex);
      return m_bWaiting;
    }

  private:    
    bool                         m_bWaiting;     /**< true while a thread is waiting or when it hasn't started waiting yet */
    PLATFORM::CCondition<bool>   m_condition;    /**< the condition to wait on */
    PLATFORM::CMutex             m_mutex;        /**< mutex for changes to this class */
    int                          m_retval;
    bool                         m_bSucceeded;
    uint8_t                      m_addrs;
    cec_opcode                   m_opcode;
  };
 
};
