/**
 * @file message_center.c
 * @brief 实现板内主题注册、发布和订阅消息队列。
 */

#include "message_center.h"
#include "main.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

/* message_center是fake head node,是方便链表编写的技巧,这样就不需要处理链表头的特殊情况 */
static Publisher_t message_center = {
	.topic_name      = "Message_Manager",
	.first_subs      = NULL,
	.next_topic_node = NULL
};

/** @brief 检查主题名是否能放入固定长度的名称缓冲区。 */
static void CheckName(char* name)
{
	if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
	{
		LOGERROR("EVENT NAME TOO LONG:%s", name);
		Error_Handler();
	}
}

/** @brief 确保同名主题的发布者和订阅者使用相同的数据长度。 */
static void CheckLen(uint8_t len1, uint8_t len2)
{
	if (len1 != len2)
	{
		LOGERROR("EVENT LEN NOT SAME:%d,%d", len1, len2);
		Error_Handler();
	}
}

/** @brief 查找或创建主题结点，并标记发布者已注册。 */
Publisher_t* PubRegister(char* name, uint8_t data_len)
{
	CheckName(name);
	Publisher_t* node = &message_center;
	while (node->next_topic_node) // message_center会直接跳过,不需要特殊处理,这被称作dumb_head(编程技巧)
	{
		node = node->next_topic_node;            // 切换到下一个发布者(话题)结点
		if (strcmp(node->topic_name, name) == 0) // 如果已经注册了相同的话题,直接返回结点指针
		{
			CheckLen(data_len, node->data_len);
			node->pub_registered_flag = 1;
			return node;
		}
	} // 遍历完发现尚未创建name对应的话题
	// 在链表尾部创建新的话题并初始化
	node->next_topic_node = (Publisher_t*)malloc(sizeof(Publisher_t));
	if (node->next_topic_node == NULL)
		Error_Handler();
	memset(node->next_topic_node, 0, sizeof(Publisher_t));
	node->next_topic_node->data_len = data_len;
	strcpy(node->next_topic_node->topic_name, name);
	node->next_topic_node->pub_registered_flag = 1;
	return node->next_topic_node;
}

/** @brief 为指定主题创建独立订阅队列并接入订阅者链表。 */
Subscriber_t* SubRegister(char* name, uint8_t data_len)
{
	Publisher_t*  pub = PubRegister(name, data_len); // 查找或创建该话题的发布者
	// 创建新的订阅者结点,申请内存,注意要memset因为新空间不一定是空的,可能有之前留存的垃圾值
	Subscriber_t* ret = (Subscriber_t*)malloc(sizeof(Subscriber_t));
	if (ret == NULL)
		Error_Handler();
	memset(ret, 0, sizeof(Subscriber_t));
	// 对新建的Subscriber进行初始化
	ret->data_len = data_len; // 设定数据长度
	for (size_t i = 0; i < QUEUE_SIZE; ++i)
	{
		// 给消息队列的每一个元素分配空间,queue里保存的实际上是数据执指针,这样可以兼容不同的数据长度
		ret->queue[i] = malloc(data_len);
		if (ret->queue[i] == NULL)
			Error_Handler();
	}
	// 如果是第一个订阅者,特殊处理一下,将first_subs指针指向新建的订阅者(详见文档)
	if (pub->first_subs == NULL)
	{
		pub->first_subs = ret;
		return ret;
	}
	// 若该话题已经有订阅者, 遍历订阅者链表,直到到达尾部
	Subscriber_t* sub = pub->first_subs; // 作为iterator
	while (sub->next_subs_queue)         // 遍历订阅了该话题的订阅者链表
	{
		sub = sub->next_subs_queue; // 移动到下一个订阅者,遇到空指针停下,说明到了链表尾部
	}
	sub->next_subs_queue = ret; // 把刚刚创建的订阅者接上
	return ret;
}

/** @brief 从订阅队列取出最早的一条消息；队列空时返回 0。 */
uint8_t SubGetMessage(Subscriber_t* sub, void* data_ptr)
{
	taskENTER_CRITICAL();
	if (sub->temp_size == 0)
	{
		taskEXIT_CRITICAL();
		return 0;
	}
	memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
	sub->front_idx = (sub->front_idx + 1) % QUEUE_SIZE; // 队列头索引增加
	sub->temp_size--;                                   // pop一个数据,长度减1
	taskEXIT_CRITICAL();
	return 1;
}

/** @brief 将消息复制给每个订阅者；队列满时覆盖最早的消息。 */
uint8_t PubPushMessage(Publisher_t* pub, void* data_ptr)
{
	Subscriber_t* iter = pub->first_subs; // 局部变量避免不同任务发布时互相覆盖遍历位置
	taskENTER_CRITICAL();
	// 遍历订阅了当前话题的所有订阅者,依次填入最新消息
	while (iter)
	{
		if (iter->temp_size == QUEUE_SIZE) // 如果队列已满,则需要删除最老的数据(头部),再填入
		{
			// 队列头索引前移动,相当于抛弃前一个位置的数据,被抛弃的位置稍后会被写入新的数据
			iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
			iter->temp_size--; // 相当于出队,size-1
		}
		// 将Pub的数据复制到队列的尾部(最新)
		memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
		iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE; // 队列尾部前移
		iter->temp_size++;                                  // 入队,size+1

		iter = iter->next_subs_queue; // 访问下一个订阅者
	}
	taskEXIT_CRITICAL();
	return 1;
}
