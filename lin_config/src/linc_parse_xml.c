#include <libxml/parser.h>
#include <assert.h>
#include <string.h>
#include "lin_config.h"

static inline int linc_xml_get_prop_int(xmlNodePtr cur, const xmlChar* str)
{
	int val;
	xmlChar *attr;

	attr = xmlGetProp(cur, str);
	if (!attr)
		assert(0);

	val = atoi((const char *)attr); // FIXME error handling
	xmlFree(attr);

	return val;
}

static inline int linc_xml_get_element_int(xmlDocPtr doc, xmlNodePtr cur)
{
	xmlChar *key;
	int val;

	key = xmlNodeListGetString(doc, cur->children, 1);
	if (!key)
		assert(0);

	val = atoi((const char *)key); // FIXME error handling etc.
	xmlFree(key);

	return val;
}

void linc_parse_scheduler_entries(struct linc_lin_state *linc_lin_state, xmlDocPtr doc, xmlNodePtr cur)
{
	cur = cur->children;
	while (cur) {
		if (!xmlStrcmp(cur->name, (const xmlChar *)"Entry")) {
			int linid;
			int interval;
			linid = linc_xml_get_element_int(doc, cur);
			interval = linc_xml_get_prop_int(cur, (const xmlChar *)"Time");

			linc_lin_state->scheduler_entry[linc_lin_state->scheduler_entries_cnt].lin_id = linid;
			linc_lin_state->scheduler_entry[linc_lin_state->scheduler_entries_cnt].interval_ms = interval;
			linc_lin_state->scheduler_entries_cnt++;

			//printf("Time = %d Entry = %d\n", interval, linid);
		}
		cur = cur->next;
	}
}

void linc_parse_frame_configuration(struct linc_lin_state *linc_lin_state, xmlDocPtr doc, xmlNodePtr cur)
{
	xmlNodePtr tmp_node;
	int val;

	cur = cur->children;
	while (cur) {
		if (!xmlStrcmp(cur->name, (const xmlChar *)"Frame")) {
			tmp_node = cur->children;
			/* We are able to write into the main Configuration array after
			parsing of all necessary elements (especially LIN ID) -- store
			parsed elements in this temporary entry -- copy the entry afterwards */
			struct linc_frame_entry tmp_fr_entry;
			int linid = -1;

			while (tmp_node) {
				if (!xmlStrcmp(tmp_node->name, (const xmlChar *)"ID")) {
					val = linc_xml_get_element_int(doc, tmp_node);
					linid = val;
					//printf("ID = %d\n", val);
				}
				if (!xmlStrcmp(tmp_node->name, (const xmlChar *)"Length")) {
					val = linc_xml_get_element_int(doc, tmp_node);
					tmp_fr_entry.data_len = val;
					//printf("Length = %d\n", val);
				}
				if (!xmlStrcmp(tmp_node->name, (const xmlChar *)"Active")) {
					val = linc_xml_get_element_int(doc, tmp_node);
					tmp_fr_entry.status = val;
					//printf("Active = %d\n", val);
				}
				if (!xmlStrcmp(tmp_node->name, (const xmlChar *)"Data")) {
					int indx = 0;
					xmlNodePtr tmp_node2;
					tmp_node2 = tmp_node->children;
					while (tmp_node2) {
						if (!xmlStrcmp(tmp_node2->name, (const xmlChar *)"Byte")) {
							// Byte indexing in XML file is wrong
							//indx = linc_xml_get_prop_int(tmp_node2,
							//	(const xmlChar *)"Index");
							val = linc_xml_get_element_int(doc, tmp_node2);
							//printf("Data = %d\n", val);
							snprintf((char *)&tmp_fr_entry.data[indx], 1, "%i", val);
							indx++;
						}
						tmp_node2 = tmp_node2->next;
					}
				}
				tmp_node = tmp_node->next;
			}

			if (linid >= 0) {
				memcpy(&linc_lin_state->frame_entry[linid], &tmp_fr_entry,
					sizeof(struct linc_frame_entry));
			}
		}
		cur = cur->next;
	}
}

int linc_parse_configuration(char *filename, struct linc_lin_state *linc_lin_state)
{
	xmlDocPtr doc;
	xmlNodePtr cur_node;

	if (!filename)
		filename = PCL_DEFAULT_CONFIG;

	xmlKeepBlanksDefault(1);
	doc = xmlParseFile(filename);
	if (doc == NULL)
		return -1;

	cur_node = xmlDocGetRootElement(doc);
	if (cur_node == NULL) {
		fprintf(stderr, "Configuration file %s is empty\n", filename);
		xmlFreeDoc(doc);
		return -1;
	}

	/* Check for Root element */
	if (xmlStrcmp(cur_node->name, (const xmlChar *)"PCLIN_PROFILE"))
		goto exit_failure;

	/* Check for LIN element */
	cur_node = cur_node->children;
	while (cur_node) {
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"LIN"))
			break;

		cur_node = cur_node->next;
	}

	if (!cur_node)
		goto exit_failure;

	/* Process LIN configuration */
	cur_node = cur_node->children;
	while (cur_node) {
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Active")) {
			linc_lin_state->is_active = linc_xml_get_element_int(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Baudrate")) {
			linc_lin_state->baudrate = linc_xml_get_element_int(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Master_Status")) {
			linc_lin_state->master_status = linc_xml_get_element_int(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Bus_Termination")) {
			linc_lin_state->bus_termination = linc_xml_get_element_int(doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Scheduler_Entries")) {
			linc_parse_scheduler_entries(linc_lin_state, doc, cur_node);
		}
		if (!xmlStrcmp(cur_node->name, (const xmlChar *)"Frame_Configuration")) {
			linc_parse_frame_configuration(linc_lin_state, doc, cur_node);
		}

		cur_node = cur_node->next;
	}

	xmlFreeDoc(doc);
	return 0;

exit_failure:
	fprintf(stderr, "Invalid configuration file\n");
	xmlFreeDoc(doc);
	return -1;
}

