/**
* @file DebugLog.h
* @brief �f�o�b�O����
* @author f.aimano
* @date 2016/09/09
*/

#pragma once

/** ���O�o�͂�ݒ�(��`�l���L���ȏꍇ�A���O���o�͂���) */
#define LOG_FATAL_ON	//<! �v���I�ȃG���[���O�̏o�͐ݒ�	�F�펞ON
#define LOG_ERROR_ON	//<! �G���[���O�̏o�͐ݒ�			�F�펞ON
#define LOG_WARN_ON		//<! �x���̏o�͐ݒ�					�F�펞ON
#define LOG_INFO_ON		//<! ��񃍃O�̏o�͐ݒ�				�F�펞ON
//#define LOG_DEBUG_ON	//<! �f�o�b�O���O�̏o�͐ݒ�			�F�펞OFF

#if defined(LOG_FATAL_ON)
#define LOG_FATAL(...)	printf("[FATAL] "); printf(__VA_ARGS__);	//!< �v���I�ȃG���[���O�o��
#else
#define LOG_FATAL(...)	//!< �v���I�ȃG���[���O�o��
#endif

#if defined(LOG_ERROR_ON)
#define LOG_ERROR(...)	printf("[ERROR] "); printf(__VA_ARGS__);	//!< �G���[���O�o��
#else
#define LOG_ERROR(...)	//!< �G���[���O�o��
#endif

#if defined(LOG_WARN_ON)
#define LOG_WARN(...)	printf("[WARN ] "); printf(__VA_ARGS__);	//!< �x�����O�o��
#else
#define LOG_WARN(...)	//!< �x�����O�o��
#endif

#if defined(LOG_INFO_ON)
#define LOG_INFO(...)	printf("[INFO ] "); printf(__VA_ARGS__);	//!< ��񃍃O�o��
#else
#define LOG_INFO(...)	//!< ��񃍃O�o��
#endif

#if defined(LOG_DEBUG_ON)
#define LOG_DEBUG(...)  printf("[DEBUG] "); printf(__VA_ARGS__);	//!< �f�o�b�O���O�o��
#else
#define LOG_DEBUG(...)	//!< �f�o�b�O���O�o��
#endif