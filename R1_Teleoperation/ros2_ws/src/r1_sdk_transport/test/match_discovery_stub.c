/* Offline LD_PRELOAD fixture for the observer's read-only discovery queries.
 *
 * This fixture creates no DDS entity and no publisher.  It replaces only the
 * three discovery/status getters (and their matching deallocator) so process
 * tests can exercise stable and churning evidence without a robot or network
 * writer.  Never install or use it outside tests.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef int32_t dds_entity_t;
typedef int32_t dds_return_t;
typedef uint64_t dds_instance_handle_t;

typedef struct dds_guid
{
  unsigned char value[16];
} dds_guid_t;

typedef struct dds_subscription_matched_status
{
  uint32_t total_count;
  int32_t total_count_change;
  uint32_t current_count;
  int32_t current_count_change;
  dds_instance_handle_t last_publication_handle;
} dds_subscription_matched_status_t;

typedef struct dds_builtintopic_endpoint
{
  dds_guid_t key;
  dds_guid_t participant_key;
  dds_instance_handle_t participant_instance_handle;
  char * topic_name;
  char * type_name;
  void * qos;
} dds_builtintopic_endpoint_t;

static unsigned int status_calls;

static uint32_t configured_count(void)
{
  const char * value = getenv("R1_MATCH_STUB_EXPECTED");
  return value != NULL && strcmp(value, "2") == 0 ? 2U : 1U;
}

static int churn_enabled(void)
{
  const char * value = getenv("R1_MATCH_STUB_MODE");
  return value != NULL && strcmp(value, "churn") == 0;
}

dds_return_t dds_get_subscription_matched_status(
  dds_entity_t reader, dds_subscription_matched_status_t * status)
{
  (void)reader;
  if (status == NULL) {
    return -3;
  }
  ++status_calls;
  const uint32_t expected = configured_count();
  const int churned = churn_enabled() && status_calls > 5U;
  status->total_count = churned ? expected + 1U : expected;
  status->current_count = expected;
  status->total_count_change = status_calls == 1U ? (int32_t)expected :
    (churned && status_calls == 6U ? 1 : 0);
  status->current_count_change = status_calls == 1U ? (int32_t)expected : 0;
  status->last_publication_handle = expected == 2U ? 202U : 101U;
  return 0;
}

dds_return_t dds_get_matched_publications(
  dds_entity_t reader, dds_instance_handle_t * handles, size_t capacity)
{
  (void)reader;
  const uint32_t expected = configured_count();
  if (handles != NULL && capacity > 0U) {
    handles[0] = churn_enabled() && status_calls > 5U ? 303U : 101U;
    if (expected == 2U && capacity > 1U) {
      handles[1] = 202U;
    }
  }
  return (dds_return_t)expected;
}

dds_builtintopic_endpoint_t * dds_get_matched_publication_data(
  dds_entity_t reader, dds_instance_handle_t handle)
{
  (void)reader;
  (void)handle;
  dds_builtintopic_endpoint_t * endpoint = calloc(1U, sizeof(*endpoint));
  if (endpoint == NULL) {
    return NULL;
  }
  endpoint->topic_name = strdup("rt/arm_sdk");
  endpoint->type_name = strdup("unitree_hg::msg::dds_::LowCmd_");
  if (endpoint->topic_name == NULL || endpoint->type_name == NULL) {
    free(endpoint->topic_name);
    free(endpoint->type_name);
    free(endpoint);
    return NULL;
  }
  return endpoint;
}

void dds_builtintopic_free_endpoint(dds_builtintopic_endpoint_t * endpoint)
{
  if (endpoint == NULL) {
    return;
  }
  free(endpoint->topic_name);
  free(endpoint->type_name);
  free(endpoint);
}
