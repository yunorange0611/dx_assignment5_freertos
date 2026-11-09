#ifndef __TASK_RC_H__
#define __TASK_RC_H__

#ifdef __cplusplus
extern "C" {
#endif


void Task_RC_Init(void);
void Task_RC_Handle(void);
void start_task_rc(void *argument);

#ifdef __cplusplus
}
#endif
#endif