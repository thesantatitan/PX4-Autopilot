@###############################################
@#
@# EmPy template
@#
@###############################################
@# Generates publications and subscriptions for XRCE
@#
@# Context:
@#  - msgs (List) list of all RTPS messages
@#  - topics (List) list of all topic names
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import os


}@


#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
@[for include in type_includes]@
#include <uORB/ucdr/@(include).h>
@[end for]@


#define TOPIC_NAME_SIZE 128

static bool generate_topic_name(char *topic, const char *client_namespace, const char *direction, const char *name)
{
	if (client_namespace != nullptr) {
		int ret = snprintf(topic, TOPIC_NAME_SIZE, "rt/%s/fmu/%s/%s", client_namespace, direction, name);
		return (ret > 0 && ret < TOPIC_NAME_SIZE);
	}

	int ret = snprintf(topic, TOPIC_NAME_SIZE, "rt/fmu/%s/%s", direction, name);
	return (ret > 0 && ret < TOPIC_NAME_SIZE);
}

// Subscribers for messages to send
struct SendTopicsSubs {
@[    for pub in publications]@
	uORB::Subscription @(pub['topic_simple'])_sub{ORB_ID(@(pub['topic_simple']))};
	uxrObjectId @(pub['topic_simple'])_data_writer{};
@[    end for]@

	uint32_t num_payload_sent{};

	void update(uxrSession *session, uxrStreamId create_stream_id, uxrStreamId publish_stream_id, uxrObjectId participant_id, const char *client_namespace);
};

static bool create_data_writer(uxrSession *session, uxrStreamId create_stream_id, uxrObjectId participant_id, uint16_t id, const char *client_namespace, const char* topic_name_simple, const char* type_name, uxrObjectId& datawriter_id)
{
	// topic
	char topic_name[TOPIC_NAME_SIZE];
	if (!generate_topic_name(topic_name, client_namespace, "out", topic_name_simple)) {
		PX4_ERR("topic path too long");
		return false;
	}

	uxrObjectId topic_id = uxr_object_id(id, UXR_TOPIC_ID);
	uint16_t topic_req = uxr_buffer_create_topic_bin(session, create_stream_id, topic_id, participant_id, topic_name, type_name, UXR_REPLACE);


	// publisher
	uxrObjectId publisher_id = uxr_object_id(id, UXR_PUBLISHER_ID);
	uint16_t publisher_req = uxr_buffer_create_publisher_bin(session, create_stream_id, publisher_id, participant_id, UXR_REPLACE);


	// data writer
	datawriter_id = uxr_object_id(id, UXR_DATAWRITER_ID);

	uxrQoS_t qos = {
		.durability = UXR_DURABILITY_TRANSIENT_LOCAL,
		.reliability = UXR_RELIABILITY_BEST_EFFORT,
		.history = UXR_HISTORY_KEEP_LAST,
		.depth = 0
	};

	uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(session, create_stream_id, datawriter_id, publisher_id, topic_id, qos, UXR_REPLACE);

	// Send create entities message and wait its status
	uint16_t requests[3] {topic_req, publisher_req, datawriter_req};
	uint8_t status[3];

	if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
		PX4_ERR("create entities failed: %s, topic: %i publisher: %i datawriter: %i",
			topic_name, status[0], status[1], status[2]);
		return false;

	} else {
		PX4_INFO("successfully created %s data writer, topic id: %d", topic_name, topic_id.id);
	}

	return true;
}

void SendTopicsSubs::update(uxrSession *session, uxrStreamId create_stream_id, uxrStreamId publish_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
	int64_t time_offset_us = session->time_offset / 1000; // ns -> us
@[    for idx, pub in enumerate(publications)]@
	{
		@(pub['simple_base_type'])_s data;

		if (@(pub['topic_simple'])_sub.update(&data)) {

			if (@(pub['topic_simple'])_data_writer.id == UXR_INVALID_ID) {
				// data writer not created yet
				create_data_writer(session, create_stream_id, participant_id, @(idx) + 1, client_namespace, "@(pub['topic_simple'])", "@(pub['dds_type'])", @(pub['topic_simple'])_data_writer);
			}

			ucdrBuffer ub;
			uint32_t topic_size = ucdr_topic_size_@(pub['simple_base_type'])();
			uxr_prepare_output_stream(session, publish_stream_id, @(pub['topic_simple'])_data_writer, &ub, topic_size);
			ucdr_serialize_@(pub['simple_base_type'])(data, ub, time_offset_us);
			// TODO: fill up the MTU and then flush, which reduces the packet overhead
			uxr_flash_output_streams(session);
			num_payload_sent += topic_size;
		}
	}
@[    end for]@
}

// Publishers for received messages
struct RcvTopicsPubs {
@[    for sub in subscriptions]@
	uORB::Publication<@(sub['simple_base_type'])_s> @(sub['topic_simple'])_pub{ORB_ID(@(sub['topic_simple']))};
@[    end for]@

	uxrSession *session;

	uint32_t num_payload_received{};

	bool init(uxrSession *session_, uxrStreamId stream_id, uxrStreamId input_stream, uxrObjectId participant_id, const char *client_namespace);
};

static bool create_data_reader(uxrSession *session, uxrStreamId stream_id, uxrStreamId input_stream, uxrObjectId participant_id, uint16_t index, const char *client_namespace, const char* topic_name_simple, const char* type_name)
{
	// topic
	char topic_name[TOPIC_NAME_SIZE];

	if (!generate_topic_name(topic_name, client_namespace, "in", topic_name_simple)) {
		PX4_ERR("topic path too long");
		return false;
	}

	uint16_t id = index + 1000;


	uxrObjectId topic_id = uxr_object_id(id, UXR_TOPIC_ID);
	uint16_t topic_req = uxr_buffer_create_topic_bin(session, stream_id, topic_id, participant_id, topic_name, type_name, UXR_REPLACE);


	// subscriber
	uxrObjectId subscriber_id = uxr_object_id(id, UXR_SUBSCRIBER_ID);
	uint16_t subscriber_req = uxr_buffer_create_subscriber_bin(session, stream_id, subscriber_id, participant_id, UXR_REPLACE);


	// data reader
	uxrObjectId datareader_id = uxr_object_id(id, UXR_DATAREADER_ID);

	uxrQoS_t qos = {
		.durability = UXR_DURABILITY_VOLATILE,
		.reliability = UXR_RELIABILITY_BEST_EFFORT,
		.history = UXR_HISTORY_KEEP_LAST,
		.depth = 0
	};

	uint16_t datareader_req = uxr_buffer_create_datareader_bin(session, stream_id, datareader_id, subscriber_id, topic_id, qos, UXR_REPLACE);

	uint16_t requests[3] {topic_req, subscriber_req, datareader_req};
	uint8_t status[3];

	if (!uxr_run_session_until_all_status(session, 1000, requests, status, 3)) {
		PX4_ERR("create entities failed: %s %i %i %i", topic_name,
			status[0], status[1], status[2]);
		return false;
	}

	uxrDeliveryControl delivery_control{};
	delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
	uxr_buffer_request_data(session, stream_id, datareader_id, input_stream, &delivery_control);

	return true;
}

static void on_topic_update(uxrSession *session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id,
		     struct ucdrBuffer *ub, uint16_t length, void *args)
{
	RcvTopicsPubs *pubs = (RcvTopicsPubs *)args;
	const int64_t time_offset_us = session->time_offset / 1000; // ns -> us
	pubs->num_payload_received += length;

	switch (object_id.id) {
@[    for idx, sub in enumerate(subscriptions)]@
	case @(idx)+1000: {
			@(sub['simple_base_type'])_s data;

			if (ucdr_deserialize_@(sub['simple_base_type'])(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(@(sub['simple_base_type'])), data);
				pubs->@(sub['topic_simple'])_pub.publish(data);
			}
		}
		break;

@[    end for]@

	default:
		PX4_ERR("unknown object id: %i", object_id.id);
		break;
	}
}

bool RcvTopicsPubs::init(uxrSession *session_, uxrStreamId stream_id, uxrStreamId input_stream, uxrObjectId participant_id, const char *client_namespace)
{
	session = session_;

@[    for idx, sub in enumerate(subscriptions)]@
	create_data_reader(session, stream_id, input_stream, participant_id, @(idx), client_namespace, "@(sub['topic_simple'])", "@(sub['dds_type'])");
@[    end for]@

	uxr_set_topic_callback(session, on_topic_update, this);

	return true;
}
