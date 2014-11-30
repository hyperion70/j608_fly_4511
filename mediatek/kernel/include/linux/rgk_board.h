#ifndef __RGK_BOARD_H__
#define __RGK_BOARD_H__

#define RGK_ATTR_CT		", \r\n{};\t"
extern int rgk_board_properties(struct attribute_group *rgk_projects_attr_group);
extern size_t rgk_attr_data_analyse(unsigned char *buf_out, size_t n_out, const char *buf, size_t n);

#endif
