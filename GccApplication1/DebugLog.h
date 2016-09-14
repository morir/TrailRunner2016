/**
* @file DebugLog.h
* @brief デバッグ処理
* @author f.aimano
* @date 2016/09/09
*/

#pragma once

/** ログ出力を設定(定義値が有効な場合、ログを出力する) */
#define LOG_FATAL_ON	//<! 致命的なエラーログの出力設定	：常時ON
#define LOG_ERROR_ON	//<! エラーログの出力設定			：常時ON
#define LOG_WARN_ON		//<! 警告の出力設定					：常時ON
#define LOG_INFO_ON		//<! 情報ログの出力設定				：常時ON
//#define LOG_DEBUG_ON	//<! デバッグログの出力設定			：常時OFF

#if defined(LOG_FATAL_ON)
#define LOG_FATAL(...)	printf("[FATAL] "); printf(__VA_ARGS__);	//!< 致命的なエラーログ出力
#else
#define LOG_FATAL(...)	//!< 致命的なエラーログ出力
#endif

#if defined(LOG_ERROR_ON)
#define LOG_ERROR(...)	printf("[ERROR] "); printf(__VA_ARGS__);	//!< エラーログ出力
#else
#define LOG_ERROR(...)	//!< エラーログ出力
#endif

#if defined(LOG_WARN_ON)
#define LOG_WARN(...)	printf("[WARN ] "); printf(__VA_ARGS__);	//!< 警告ログ出力
#else
#define LOG_WARN(...)	//!< 警告ログ出力
#endif

#if defined(LOG_INFO_ON)
#define LOG_INFO(...)	printf("[INFO ] "); printf(__VA_ARGS__);	//!< 情報ログ出力
#else
#define LOG_INFO(...)	//!< 情報ログ出力
#endif

#if defined(LOG_DEBUG_ON)
#define LOG_DEBUG(...)  printf("[DEBUG] "); printf(__VA_ARGS__);	//!< デバッグログ出力
#else
#define LOG_DEBUG(...)	//!< デバッグログ出力
#endif